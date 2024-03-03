//! Near Field Communication Tag (NFCT) driver.

#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::RangeInclusive;
use core::task::Poll;

use embassy_hal_internal::{into_ref, PeripheralRef};

use crate::chip::FORCE_COPY_BUFFER_SIZE;
use crate::interrupt::typelevel::Interrupt;
use crate::util::slice_in_ram_or;
use crate::{interrupt, Peripheral};

/// NFCT error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// EasyDMA can only read from data memory, read only buffers in flash will fail.
    BufferNotInRAM,
    /// Write not triggered before timeout
    FrameDelayTimeout,
    /// CRC received does not match local check
    Crc,
    /// Frame received with parity error
    Parity,
    /// Received frame larger than provided buffer
    Overrun,
    /// NFC Field was lost during transaction
    FieldLost,
    /// Invalid value supplied in argument
    BadArgument,
    /// Some other error
    Other,
}

/// NFCT driver.
pub struct Nfct<'d, T: Instance> {
    _p: PeripheralRef<'d, T>,
}

/// NFCT config
#[non_exhaustive]
pub struct Config {
    /// Frame delay settings
    pub frame_delay: FrameDelayConfig,
    /// TX frame config
    pub tx: FrameConfig,
    /// TX frame discard mode
    pub tx_discard_mode: FrameDiscardMode,
    /// RX frame config
    pub rx: FrameConfig,
    /// REQA/SENS_REQ response.
    /// If `id` is supplied, bits 6-7 is ignored.
    /// If bits 6-7 is `0b11`, it is set to `0b00` even if `id` is `None`.
    pub atqa: u16,
    /// Select/SEL_REQ response.
    /// Bit 2 is ignored by hardware.
    pub sak: u8,
    /// NFCID1 value. If None, takes from FICR.
    pub id: Option<NfcId>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frame_delay: Default::default(),
            tx: Default::default(),
            tx_discard_mode: Default::default(),
            rx: Default::default(),
            atqa: 1,
            sak: 0,
            id: None,
        }
    }
}

/// Frame TX/RX config
#[non_exhaustive]
pub struct FrameConfig {
    /// Parity added/expected in the frame
    pub add_parity: bool,
    /// SoF added/expected in the frame
    pub sof: bool,
    /// CRC added/expected in the frame
    pub crc: bool,
}

impl Default for FrameConfig {
    fn default() -> Self {
        Self {
            add_parity: true,
            sof: true,
            crc: true,
        }
    }
}

/// Where to discard bits from for non-whole-byte transmissions
#[non_exhaustive]
#[derive(Default)]
pub enum FrameDiscardMode {
    #[default]
    Start,
    End,
}

/// NFCT frame delay config. All delays are in number of 13.56 MHz clocks.
#[non_exhaustive]
pub enum FrameDelayConfig {
    /// Transmission is independent of frame timer and will start when the STARTTX task is triggered. No timeout.
    FreeRun,
    /// Frame is transmitted in the range specified.
    Window(RangeInclusive<u16>),
    /// Frame is transmitted exactly at the delay specified.
    ExactVal(u16),
    /// Frame is transmitted on a bit grid in the range specified.
    WindowGrid(RangeInclusive<u16>),
}

impl Default for FrameDelayConfig {
    fn default() -> Self {
        FrameDelayConfig::Window(0x00000480..=0x00001000)
    }
}

/// NFCT automatic collision resolution ID
#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum NfcId {
    FourBytes([u8; 4]),
    SevenBytes([u8; 7]),
    TenBytes([u8; 10]),
}

impl Default for NfcId {
    fn default() -> Self {
        NfcId::FourBytes([0x00, 0x00, 0x63, 0x63])
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();

        if r.events_selected.read().bits() != 0 {
            s.selected.wake();
            r.intenclr.write(|w| w.selected().clear());
        }

        if r.events_rxframeend.read().bits() != 0
            || r.events_rxerror.read().bits() != 0
            || r.events_fieldlost.read().bits() != 0
        {
            s.rx_finished.wake();
            r.intenclr
                .write(|w| w.rxframeend().clear().rxerror().clear().fieldlost().clear());
        }

        if r.events_txframeend.read().bits() != 0
            || r.events_error.read().bits() != 0
            || r.events_fieldlost.read().bits() != 0
        {
            s.tx_finished.wake();
            r.intenclr
                .write(|w| w.txframeend().clear().error().clear().fieldlost().clear());
        }
    }
}

impl<'d, T: Instance> Nfct<'d, T> {
    /// Create a new NFCT.
    pub fn new(
        nfct: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(nfct);
        let r = T::regs();

        #[cfg(not(feature = "nrf52832"))]
        r.autocolresconfig.write(|w| w.mode().set_bit());

        let id = config.id.unwrap_or_else(|| {
            let ficr = crate::pac::FICR::ptr() as *const u8;
            // SAFETY: All nRF models with NFCT seems to have NFC.TAGHEADER0-3 at an offset of 0x450.
            // NFC.TAGHEADER0-3 are also all read-only, so nobody can write to them while we're holding the slice.
            let id = unsafe { core::slice::from_raw_parts(ficr.offset(0x450), 16) };
            let size = (config.atqa >> 6) & 0b11;
            match size {
                1 => NfcId::SevenBytes(id[0..7].try_into().unwrap()),
                2 => NfcId::TenBytes(id[0..10].try_into().unwrap()),
                _ => NfcId::FourBytes(id[0..4].try_into().unwrap()),
            }
        });

        match config.frame_delay {
            FrameDelayConfig::FreeRun => {
                r.framedelaymode.write(|w| w.framedelaymode().free_run());
            }
            FrameDelayConfig::Window(range) => {
                r.framedelaymin.write(|w| w.framedelaymin().variant(*range.start()));
                r.framedelaymax.write(|w| w.framedelaymax().variant(*range.end() as _));
                r.framedelaymode.write(|w| w.framedelaymode().window());
            }
            FrameDelayConfig::ExactVal(delay) => {
                r.framedelaymax.write(|w| w.framedelaymax().variant(delay as _));
                r.framedelaymode.write(|w| w.framedelaymode().exact_val());
            }
            FrameDelayConfig::WindowGrid(range) => {
                r.framedelaymin.write(|w| w.framedelaymin().variant(*range.start()));
                r.framedelaymax.write(|w| w.framedelaymax().variant(*range.end() as _));
                r.framedelaymode.write(|w| w.framedelaymode().window_grid());
            }
        }

        match id {
            NfcId::FourBytes(id) => {
                r.nfcid1_last.write(|w| {
                    w.nfcid1_w()
                        .variant(id[0])
                        .nfcid1_x()
                        .variant(id[1])
                        .nfcid1_y()
                        .variant(id[2])
                        .nfcid1_z()
                        .variant(id[3])
                });
            }
            NfcId::SevenBytes(id) => {
                r.nfcid1_2nd_last.write(|w| {
                    w.nfcid1_t()
                        .variant(id[0])
                        .nfcid1_u()
                        .variant(id[1])
                        .nfcid1_v()
                        .variant(id[2])
                });
                r.nfcid1_last.write(|w| {
                    w.nfcid1_w()
                        .variant(id[3])
                        .nfcid1_x()
                        .variant(id[4])
                        .nfcid1_y()
                        .variant(id[5])
                        .nfcid1_z()
                        .variant(id[6])
                });
            }
            NfcId::TenBytes(id) => {
                r.nfcid1_3rd_last.write(|w| {
                    w.nfcid1_q()
                        .variant(id[0])
                        .nfcid1_r()
                        .variant(id[1])
                        .nfcid1_s()
                        .variant(id[2])
                });
                r.nfcid1_2nd_last.write(|w| {
                    w.nfcid1_t()
                        .variant(id[3])
                        .nfcid1_u()
                        .variant(id[4])
                        .nfcid1_v()
                        .variant(id[5])
                });
                r.nfcid1_last.write(|w| {
                    w.nfcid1_w()
                        .variant(id[6])
                        .nfcid1_x()
                        .variant(id[7])
                        .nfcid1_y()
                        .variant(id[8])
                        .nfcid1_z()
                        .variant(id[9])
                });
            }
        }

        // SAFETY: sak is u8. We're only writing to the actual 8 bits, which are all safe to write whatever to.
        r.selres.write(|w| unsafe { w.bits(config.sak as u32) });

        let atqa = config.atqa & !0b11000000;
        let atqa = atqa
            | (match id {
                NfcId::FourBytes(_) => 0,
                NfcId::SevenBytes(_) => 1,
                NfcId::TenBytes(_) => 2,
            } << 6);
        // SAFETY: atqa is u16, and the only two bits with valid values are sanitized above.
        r.sensres.write(|w| unsafe { w.bits(atqa as u32) });

        r.intenclr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self { _p: nfct }
    }

    /// Wait until we're selected for reading by a reader.
    pub async fn wait_for_reader(&mut self) {
        let r = T::regs();
        r.events_selected.reset();
        r.intenset.write(|w| w.selected().set());
        r.shorts
            .write(|w| w.fielddetected_activate().set_bit().fieldlost_sense().set_bit());
        r.tasks_sense.write(|w| w.tasks_sense().set_bit());
        poll_fn(|cx| {
            T::state().selected.register(cx.waker());
            if T::regs().events_selected.read().bits() != 0 {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
        r.events_selected.reset();
    }

    /// Reads a frame from the reader into the provided buffer.
    /// On success, returns the amount of bytes read plus the amount of bits not aligned to bytes.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<(usize, usize), Error> {
        let r = T::regs();
        r.events_rxframeend.reset();
        r.events_rxerror.reset();
        r.events_fieldlost.reset();
        r.intenset
            .write(|w| w.rxframeend().set().rxerror().set().fieldlost().set());
        r.packetptr.write(|w| w.ptr().variant(buf.as_ptr() as u32));
        r.maxlen.write(|w| w.maxlen().variant(buf.len().min(255) as _));
        r.tasks_enablerxdata.write(|w| w.tasks_enablerxdata().set_bit());
        poll_fn(|cx| {
            T::state().rx_finished.register(cx.waker());
            if T::regs().events_rxframeend.read().bits() != 0
                || T::regs().events_rxerror.read().bits() != 0
                || T::regs().events_fieldlost.read().bits() != 0
            {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
        if r.events_rxframeend.read().bits() != 0 {
            let bytes = r.rxd.amount.read().rxdatabytes().bits();
            let bits = r.rxd.amount.read().rxdatabits().bits();
            r.events_rxframeend.reset();
            return Ok((bytes as usize, bits as usize));
        }
        if r.events_rxerror.read().bits() != 0 {
            r.events_rxerror.reset();
            if r.framestatus.rx.read().crcerror().is_crcerror() {
                r.framestatus.rx.write(|w| {
                    w.crcerror()
                        .clear_bit_by_one()
                        .paritystatus()
                        .clear_bit_by_one()
                        .overrun()
                        .clear_bit_by_one()
                });
                return Err(Error::Crc);
            }
            if r.framestatus.rx.read().paritystatus().is_parity_error() {
                r.framestatus.rx.write(|w| {
                    w.crcerror()
                        .clear_bit_by_one()
                        .paritystatus()
                        .clear_bit_by_one()
                        .overrun()
                        .clear_bit_by_one()
                });
                return Err(Error::Parity);
            }
            if r.framestatus.rx.read().overrun().is_overrun() {
                r.framestatus.rx.write(|w| {
                    w.crcerror()
                        .clear_bit_by_one()
                        .paritystatus()
                        .clear_bit_by_one()
                        .overrun()
                        .clear_bit_by_one()
                });
                return Err(Error::Overrun);
            }
        }
        if r.events_fieldlost.read().bits() != 0 {
            r.events_fieldlost.reset();
            return Err(Error::FieldLost);
        }
        Err(Error::Other)
    }

    /// Writes a frame from the provided buffer to the reader. Panicks if the buffer isn't in RAM.
    pub async fn write_from_ram(&mut self, buf: &[u8], extra_bits: u8) -> Result<(), Error> {
        if extra_bits > 7 {
            return Err(Error::BadArgument)
        }
        slice_in_ram_or(buf, Error::BufferNotInRAM)?;
        let r = T::regs();
        r.events_txframeend.reset();
        r.events_error.reset();
        r.events_fieldlost.reset();
        r.intenset
            .write(|w| w.txframeend().set().error().set().fieldlost().set());
        r.packetptr.write(|w| w.ptr().variant(buf.as_ptr() as u32));
        let len = if extra_bits == 0 {
            buf.len()
        } else {
            buf.len() - 1
        };
        r.txd.amount.write(|w| w.txdatabytes().variant(len.min(255) as _).txdatabits().variant(extra_bits));
        r.tasks_starttx.write(|w| w.tasks_starttx().set_bit());
        poll_fn(|cx| {
            T::state().tx_finished.register(cx.waker());
            if T::regs().events_txframeend.read().bits() != 0
                || T::regs().events_error.read().bits() != 0
                || T::regs().events_fieldlost.read().bits() != 0
            {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
        if r.events_txframeend.read().bits() != 0 {
            r.events_txframeend.reset();
            return Ok(());
        }
        if r.events_error.read().bits() != 0 {
            r.events_error.reset();
            if r.errorstatus.read().framedelaytimeout().bit_is_set() {
                r.errorstatus.write(|w| {
                    w.framedelaytimeout()
                        .clear_bit_by_one()
                });
                return Err(Error::FrameDelayTimeout);
            }
        }
        if r.events_fieldlost.read().bits() != 0 {
            r.events_fieldlost.reset();
            return Err(Error::FieldLost);
        }
        Err(Error::Other)
    }

    /// Writes a frame from the provided buffer to the reader.
    pub async fn write(&mut self, buf: &[u8], extra_bits: u8) -> Result<(), Error> {
        match self.write_from_ram(buf, extra_bits).await {
            Err(Error::BufferNotInRAM) => {
                trace!("Copying NFCT tx buffer into RAM for DMA");
                let ram_buf = &mut [0; FORCE_COPY_BUFFER_SIZE][..buf.len()];
                ram_buf.copy_from_slice(buf);
                self.write_from_ram(ram_buf, extra_bits).await
            },
            x => x
        }
    }
}

pub(crate) mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    /// Peripheral static state
    pub struct State {
        pub selected: AtomicWaker,
        pub rx_finished: AtomicWaker,
        pub tx_finished: AtomicWaker,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                selected: AtomicWaker::new(),
                rx_finished: AtomicWaker::new(),
                tx_finished: AtomicWaker::new(),
            }
        }
    }

    pub trait Instance {
        fn regs() -> &'static crate::pac::nfct::RegisterBlock;
        fn state() -> &'static State;
    }
}

/// nfct peripheral instance.
pub trait Instance: Peripheral<P = Self> + sealed::Instance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_nfct {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl crate::nfct::sealed::Instance for peripherals::$type {
            fn regs() -> &'static crate::pac::nfct::RegisterBlock {
                unsafe { &*pac::$pac_type::ptr() }
            }
            fn state() -> &'static crate::nfct::sealed::State {
                static STATE: crate::nfct::sealed::State = crate::nfct::sealed::State::new();
                &STATE
            }
        }
        impl crate::nfct::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}
