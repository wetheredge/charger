#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use critical_section::Mutex;
use display_interface_spi::SPIInterface as SpiDisplay;
use esp_hal::clock::ClockControl;
use esp_hal::delay::Delay;
use esp_hal::gpio::{self, Io};
use esp_hal::peripherals::Peripherals;
use esp_hal::prelude::*;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::system::SystemControl;
use mipidsi::options::{Orientation, Rotation};
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

type Display<'d> = mipidsi::Display<
    SpiDisplay<
        embedded_hal_bus::spi::CriticalSectionDevice<
            'd,
            Spi<'d, esp_hal::peripherals::SPI2, esp_hal::spi::FullDuplexMode>,
            gpio::AnyOutput<'d>,
            Delay,
        >,
        gpio::AnyOutput<'d>,
    >,
    mipidsi::models::ILI9486Rgb666,
    mipidsi::NoResetPin,
>;

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

    let display = {
        let cs = gpio::AnyOutput::new(io.pins.gpio10, gpio::Level::High);
        let dc = gpio::AnyOutput::new(io.pins.gpio5, gpio::Level::Low);

        let spi = embedded_hal_bus::spi::CriticalSectionDevice::new(&spi, cs, delay).unwrap();
        let interface = SpiDisplay::new(spi, dc);

        mipidsi::Builder::new(mipidsi::models::ILI9486Rgb666, interface)
            .orientation(Orientation::new().flip_vertical().rotate(Rotation::Deg90))
            .init(&mut delay.clone())
            .unwrap()
    };

    defmt::info!("initted");

    ui::run(delay, display);
}

mod ui {
    use alloc::boxed::Box;
    use alloc::rc::Rc;
    use core::ops::Range;
    use core::time::Duration;

    use embedded_graphics::draw_target::DrawTarget;
    use esp_hal::delay::{Delay, MicrosDurationU64 as EspDuration};
    use rgb::RGB8 as SlintPixel;
    use slint::platform::software_renderer::{
        LineBufferProvider, MinimalSoftwareWindow, RepaintBufferType,
    };
    use slint::platform::WindowAdapter;
    use slint::{ComponentHandle, PlatformError};

    mod eg {
        pub(super) use embedded_graphics::geometry::{Point, Size};
        pub(super) use embedded_graphics::pixelcolor::Rgb666 as Pixel;
        pub(super) use embedded_graphics::primitives::Rectangle;
    }

    const DISPLAY_WIDTH: usize = 480;
    const DISPLAY_HEIGHT: usize = 320;

    #[allow(trivial_casts, missing_debug_implementations)]
    mod components {
        slint::include_modules!();
    }

    struct Platform {
        window: Rc<MinimalSoftwareWindow>,
    }

    impl slint::platform::Platform for Platform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
            // Only ever one window
            Ok(self.window.clone())
        }

        fn duration_since_start(&self) -> Duration {
            let duration = esp_hal::time::current_time().duration_since_epoch();
            let nanos = duration.to_nanos();
            Duration::from_nanos(nanos)
        }
    }

    struct Buffer<'display> {
        pixels: [SlintPixel; DISPLAY_WIDTH],
        display: super::Display<'display>,
    }

    impl<'display> Buffer<'display> {
        fn new(display: super::Display<'display>) -> Self {
            Self {
                pixels: [SlintPixel::new(0, 0, 0); DISPLAY_WIDTH],
                display,
            }
        }
    }

    impl LineBufferProvider for &mut Buffer<'_> {
        type TargetPixel = SlintPixel;

        fn process_line(
            &mut self,
            line: usize,
            range: Range<usize>,
            render_fn: impl FnOnce(&mut [Self::TargetPixel]),
        ) {
            let top_left = eg::Point::new(range.start as i32, line as i32);
            let width = range.end - range.start;
            let size = eg::Size::new(width as u32, 1);
            let area = eg::Rectangle::new(top_left, size);

            let slice = &mut self.pixels[0..width];
            render_fn(slice);
            self.display
                .fill_contiguous(&area, slice.iter().map(|p| eg::Pixel::new(p.r, p.g, p.b)))
                .unwrap();
        }
    }

    pub(crate) fn run(delay: Delay, display: super::Display<'_>) -> ! {
        let mut buffer = Buffer::new(display);
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        window.set_size(slint::PhysicalSize::new(
            DISPLAY_WIDTH as u32,
            DISPLAY_HEIGHT as u32,
        ));

        slint::platform::set_platform(Box::new(Platform {
            window: window.clone(),
        }))
        .unwrap();

        let ui = components::HelloWorld::new().unwrap();
        ui.show().unwrap();

        loop {
            slint::platform::update_timers_and_animations();

            window.draw_if_needed(|renderer| {
                renderer.render_by_line(&mut buffer);
            });

            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                if !window.has_active_animations() {
                    defmt::info!("duration = {:?}", duration);
                    let duration = EspDuration::micros_at_least(duration.as_micros() as u64);
                    delay.delay(duration);
                    continue;
                }
            }

            // TODO: yield
            delay.delay_millis(250);
        }
    }
}
