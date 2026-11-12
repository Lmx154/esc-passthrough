//! Finding and opening the serial port. Not used by any test: tests never open a port.

use crate::cli::PortChoice;
use crate::link::SerialLink;
use serialport::SerialPortType;
use std::time::Duration;

/// The configurator's default baud rate (`SerialDevice.vue`, the UI default); for a USB CDC port it
/// is only nominal.
pub const BAUD: u32 = 115_200;

/// Finds the port and opens it.
///
/// # Errors
///
/// A message: no port, several ports, or the open failed.
pub fn open(choice: &PortChoice) -> Result<(String, SerialLink), String> {
    let name = match choice {
        PortChoice::Path(path) => path.clone(),
        PortChoice::UsbIds(vid, pid) => {
            let ports = serialport::available_ports().map_err(|e| format!("listing ports: {e}"))?;
            let hits: Vec<String> = ports
                .into_iter()
                .filter(|p| matches!(&p.port_type, SerialPortType::UsbPort(u) if u.vid == *vid && u.pid == *pid))
                .map(|p| p.port_name)
                .collect();
            match hits.as_slice() {
                [] => {
                    return Err(format!(
                        "no port with USB ids {vid:04X}:{pid:04X}; use --port"
                    ));
                }
                [one] => one.clone(),
                many => {
                    return Err(format!(
                        "several ports with USB ids {vid:04X}:{pid:04X} ({}); use --port",
                        many.join(", ")
                    ));
                }
            }
        }
    };
    let port = serialport::new(&name, BAUD)
        .timeout(Duration::from_millis(1000))
        .open()
        .map_err(|e| format!("open {name}: {e}"))?;
    Ok((name, SerialLink(port)))
}
