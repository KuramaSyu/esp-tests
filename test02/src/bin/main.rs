#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{error, info};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Flex, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use onewire::{DS18B20, DeviceSearch, OneWire, Sensor, ds18b20};
use panic_rtt_target as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    let mut delay = esp_hal::delay::Delay::new();

    // let mut one = peripherals.GPIO3.i
    let mut onewire_pin = Flex::new(peripherals.GPIO3);

    // open drain + pull up config
    let open_drain_config = OutputConfig::default()
        .with_drive_mode(DriveMode::OpenDrain)
        .with_pull(Pull::Up);

    onewire_pin.apply_output_config(&open_drain_config); // set pin to open drain config
    onewire_pin.set_output_enable(true); // enable the config
    onewire_pin.set_input_enable(true); // enable reading
    onewire_pin.set_level(esp_hal::gpio::Level::High); // start at idle which is when pin is HIGH

    // one wire instance
    let mut onewire = OneWire::new(&mut onewire_pin, false);

    // setup http client

    loop {
        if onewire.reset(&mut delay).is_err() {
            // missing pullup or error on line
            loop {}
        }

        // search for devices
        let mut search = DeviceSearch::new();
        while let Some(device) = onewire.search_next(&mut search, &mut delay).unwrap() {
            match device.address[0] {
                ds18b20::FAMILY_CODE => {
                    let ds18b20 = DS18B20::new(device).unwrap();

                    // request sensor to measure temperature
                    let resolution = ds18b20
                        .measure_temperature(&mut onewire, &mut delay)
                        .unwrap();

                    // wait for compeltion, depends on resolution
                    delay.delay_micros(resolution.time_ms() as u32);

                    // read temperature
                    match ds18b20.read_measurement(&mut onewire, &mut delay) {
                        Ok(temperature) => info!("Temperature: {}C", temperature),
                        Err(_e) => error!("Failed to read Temerature"),
                    }
                }
                _ => {
                    // unknown device type
                }
            }
        }
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}
