//! The ESC passthrough demonstration, as a service: enumerates as a USB serial port, stays silent
//! until a configurator's first request, then serves it on the real motor-controller lines.
//! Runs until `tools/run.sh` stops it. See `hil/README.md`.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO2, USB};
use embassy_rp::pio;
use embassy_rp::usb::InterruptHandler;
use hil_common::demo;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("[INFO] passthrough_demo: start (service)");
    let p = embassy_rp::init(Default::default());
    let mut board = demo::setup(&spawner, p, Irqs);
    let failures = demo::self_check(&mut board).await;
    defmt::info!("[INFO] self-check done, {=u32} failed; serving", failures);
    demo::serve(&spawner, board);
}
