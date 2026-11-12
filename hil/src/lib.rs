//! The shared parts of the two on-target programs, and the verdict they report to the host.
#![no_std]

/// Logs the verdict and ends the `probe-rs run` session through semihosting SYS_EXIT.
/// The host exit status is the number of failed checks (at most 255): 0 is a pass.
pub fn exit(failures: u32) -> ! {
    if failures == 0 {
        defmt::info!("RESULT: PASS");
    } else {
        defmt::error!("RESULT: FAIL ({} failed checks)", failures);
    }
    semihosting::process::exit(failures.min(255) as i32)
}

pub mod demo;
