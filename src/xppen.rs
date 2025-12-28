use hidapi::HidApi;
use std::sync::mpsc;
use std::thread;

pub enum XppenMessage {
    ToggleF1,  // F1用
    ToggleF2,  // F2用
    ToggleF3,  // F3用
    ToggleF4,  // F4用
    ToggleF5,  // F5用
    ToggleF6,  // F6用
    ToggleF7,  // F7用
    ToggleF8,  // F8用
    ToggleF9,  // F9用
    ToggleF10, // F10用
    ToggleF11, // F11用
    ToggleF12, // F12用
}

pub fn start_xppen_thread(
    xppen_message_sender: mpsc::Sender<XppenMessage>,
    status_sender: mpsc::Sender<String>,
    trigger_receiver: mpsc::Receiver<()>,
) {
    thread::spawn(move || {
        // Wait for the trigger from the main UI thread
        if trigger_receiver.recv().is_err() {
            // If recv fails, it means the sender has been dropped, so we should exit.
            status_sender
                .send("XPPen thread exiting: UI thread disconnected.".to_string())
                .unwrap_or_default();
            return;
        }

        const VENDOR_ID: u16 = 0x28bd;
        const PRODUCT_ID: u16 = 0x0202;
        const F1_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F2_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F3_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F4_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F5_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F6_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F7_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F8_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F9_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F10_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F11_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        const F12_PATTERN: [u8; 32] = [
            0x02, 0xf0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];

        let max_retries = 10;
        let retry_delay = std::time::Duration::from_secs(1);
        let mut device = None;

        // Initialize HidApi once before the loop
        let api = match HidApi::new() {
            Ok(a) => a,
            Err(e) => {
                status_sender
                    .send(format!("Failed to initialize HidApi: {}", e))
                    .unwrap_or_default();
                return; // Exit thread if HidApi fails to initialize
            }
        };

        for attempt in 0..max_retries {
            match api.open(VENDOR_ID, PRODUCT_ID) {
                Ok(opened_device) => {
                    status_sender
                        .send("XPPen device connected.".to_string())
                        .unwrap_or_default();
                    device = Some(opened_device);
                    break; // Success, exit retry loop
                }
                Err(e) => {
                    let msg = format!(
                        "[Attempt {}/{}] Failed to open XPPen device: {}. Retrying...",
                        attempt + 1,
                        max_retries,
                        e
                    );
                    status_sender.send(msg).unwrap_or_default();
                }
            }
            std::thread::sleep(retry_delay);
        }

        if let Some(device) = device {
            let mut buf = [0u8; 64];
            loop {
                match device.read_timeout(&mut buf[..], 100) {
                    Ok(size) => {
                        if size == 32 {
                            let hex_string: Vec<String> = buf[..size]
                                .iter()
                                .map(|b| format!("0x{:02x}", b))
                                .collect();
                            println!(
                                "[XPPen] Received data ({} bytes): [{}]",
                                size,
                                hex_string.join(", ")
                            );
                            if &buf[..size] == F1_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF1)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F2_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF2)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F3_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF3)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F4_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF4)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F5_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF5)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F6_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF6)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F7_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF7)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F8_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF8)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F9_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF9)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F10_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF10)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F11_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF11)
                                    .unwrap_or_default();
                            } else if &buf[..size] == F12_PATTERN {
                                xppen_message_sender
                                    .send(XppenMessage::ToggleF12)
                                    .unwrap_or_default();
                            }
                        }
                    }
                    Err(e) => {
                        status_sender
                            .send(format!("XPPen device error: {}", e))
                            .unwrap_or_default();
                        break;
                    }
                }
            }
        } else {
            status_sender
                .send("Failed to connect to XPPen device after multiple attempts.".to_string())
                .unwrap_or_default();
        }
    });
}
