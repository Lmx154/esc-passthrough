//! The start-up self-check of the ESC passthrough demonstration, alone: prints `[PASS]` / `[FAIL]`
//! lines and `RESULT`, then detaches from the USB bus and exits. Needs the USB cable connected.
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
    defmt::info!("[INFO] passthrough_demo_check: start");
    let p = embassy_rp::init(Default::default());
    let mut board = demo::setup(&spawner, p, Irqs);
    let failures = demo::self_check(&mut board).await;
    demo::detach_usb();
    hil_common::exit(failures)
}
