use anyhow::Result;
use serialport::{ClearBuffer, SerialPort};
use std::io::{self, Write};
use std::time::Duration;

/// Represents a connection to the LiDAR device.
pub struct LidarConnection {
    port: Box<dyn SerialPort>,
}

impl LidarConnection {
    /// Opens a new connection to the LiDAR.
    pub fn new(path: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(path, baud_rate)
            .timeout(Duration::from_millis(100))
            .open()?;
        Ok(Self { port })
    }

    /// Sends a command and receives the response.
    pub fn send_and_receive(&mut self, command: &[u8]) -> Result<String> {
        self.port.write_all(command)?;
        self.port.clear(ClearBuffer::Input)?;
        let mut buf = Vec::new();
        let mut serial_buf = [0; 1];
        loop {
            match self.port.read(&mut serial_buf) {
                Ok(bytes) => {
                    if bytes > 0 {
                        buf.extend_from_slice(&serial_buf[..bytes]);
                        // SCIP2.0 responses are terminated by double newline.
                        if buf.ends_with(b"\n\n") {
                            break;
                        }
                    } else {
                        // No more data to read.
                        break;
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                    // Timeout is not an error, just no data available.
                    break;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(String::from_utf8_lossy(&buf).to_string())
    }
}
