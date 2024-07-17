#![no_std]
#![no_main]

extern crate alloc;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use critical_section::Mutex;
use display_interface_spi::SPIInterface as SpiDisplay;
use embedded_graphics::pixelcolor::Rgb666;
use embedded_graphics::prelude::*;
use esp_hal::clock::ClockControl;
use esp_hal::delay::Delay;
use esp_hal::gpio::{self, Io};
use esp_hal::peripherals::Peripherals;
use esp_hal::prelude::*;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::system::SystemControl;
use {esp_backtrace as _, esp_println as _};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

/// Initialize the heap
///
/// # Safety
///
/// Must be called exactly once before any allocations
unsafe fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    // SAFETY:
    // - `init_heap` is required to be called exactly once, before any allocations
    // - `HEAP_SIZE` is > 0
    unsafe { ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE) };
}

#[entry]
fn main() -> ! {
    // SAFETY: main() will only run once
    unsafe { init_heap() };

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let delay = Delay::new(&clocks);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let spi = Spi::new(peripherals.SPI2, 50.MHz(), SpiMode::Mode0, &clocks)
        .with_sck(io.pins.gpio6)
        .with_mosi(io.pins.gpio7)
        .with_miso(io.pins.gpio2);
    let spi = Mutex::new(RefCell::new(spi));

    let mut display = {
        let cs = gpio::AnyOutput::new(io.pins.gpio10, gpio::Level::High);
        let dc = gpio::AnyOutput::new(io.pins.gpio5, gpio::Level::Low);

        let spi = embedded_hal_bus::spi::CriticalSectionDevice::new(&spi, cs, delay).unwrap();
        let interface = SpiDisplay::new(spi, dc);

        mipidsi::Builder::new(mipidsi::models::ILI9486Rgb666, interface)
            .init(&mut delay.clone())
            .unwrap()
    };

    let color = [
        Rgb666::RED,
        Rgb666::BLUE,
        Rgb666::GREEN,
        Rgb666::WHITE,
        Rgb666::BLACK,
    ];
    for color in color.into_iter().cycle() {
        delay.delay(1000.millis());
        display.clear(color).unwrap();
    }

    defmt::unreachable!();
}
