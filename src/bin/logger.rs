use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

#[path = "../navigation/log_types.rs"]
mod log_types;
use log_types::{LogMessage, LogPayload};

const SOCKET_PATH: &str = "/tmp/mobitia_logger.sock";
const LOG_DIR: &str = "logs";

fn main() -> std::io::Result<()> {
    // Clean up old socket if exists
    if Path::new(SOCKET_PATH).exists() {
        fs::remove_file(SOCKET_PATH)?;
    }

    // Create logs directory
    fs::create_dir_all(LOG_DIR)?;

    let listener = UnixListener::bind(SOCKET_PATH)?;
    println!("Logger started. Listening on {}", SOCKET_PATH);

    // Accept connections (one per session usually)
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                thread::spawn(|| handle_client(stream));
            }
            Err(err) => {
                eprintln!("Error accepting connection: {}", err);
            }
        }
    }
    Ok(())
}

fn handle_client(stream: UnixStream) {
    let peer_addr = stream.peer_addr().ok();
    println!("Client connected: {:?}", peer_addr);

    let reader = BufReader::new(stream);
    let mut session_name = "unknown_session".to_string();
    let mut buffer: Vec<LogMessage> = Vec::with_capacity(100_000);
    let mut is_active = false;

    for line in reader.lines() {
        match line {
            Ok(json_str) => {
                match serde_json::from_str::<LogMessage>(&json_str) {
                    Ok(msg) => {
                        match &msg {
                            LogMessage::Start { session_name: name } => {
                                println!("Session Start: {}", name);
                                session_name = name.clone();
                                buffer.clear();
                                is_active = true;
                                // Save the start message too
                                buffer.push(msg);
                            }
                            LogMessage::Stop => {
                                println!("Session Stop: {}", session_name);
                                buffer.push(msg);
                                flush_buffer(&session_name, &buffer, "completed");
                                buffer.clear();
                                is_active = false;
                            }
                            LogMessage::Data { .. } => {
                                if !is_active {
                                    // Auto-start session if we missed the Start message
                                    println!("Data received without active session. Auto-starting.");
                                    let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
                                    session_name = format!("auto_session_{}", ts);
                                    buffer.clear();
                                    is_active = true;
                                }
                                buffer.push(msg);
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Failed to parse JSON: {} | Content: {}", e, json_str);
                    }
                }
            }
            Err(e) => {
                eprintln!("Error reading line: {}", e);
                break;
            }
        }
    }

    // Connection closed. If we were active and didn't get a Stop, it's a crash/disconnect.
    if is_active && !buffer.is_empty() {
        println!("Connection closed unexpectedly. Saving emergency log for: {}", session_name);
        flush_buffer(&session_name, &buffer, "CRASHED");
    } else {
        println!("Client disconnected.");
    }
}

fn flush_buffer(session_name: &str, buffer: &[LogMessage], suffix: &str) {
    if buffer.is_empty() {
        return;
    }

    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs();
    
    // Sanitize session name
    let safe_name: String = session_name.chars()
        .map(|c| if c.is_alphanumeric() || c == '_' || c == '-' { c } else { '_' })
        .collect();

    let filename = format!("{}/{}_{}_{}.jsonl", LOG_DIR, safe_name, timestamp, suffix);
    let path = Path::new(&filename);

    match File::create(path) {
        Ok(file) => {
            let mut writer = BufWriter::new(file);
            for msg in buffer {
                if let Ok(json) = serde_json::to_string(msg) {
                    if let Err(e) = writeln!(writer, "{}", json) {
                        eprintln!("Failed to write to log file: {}", e);
                        break;
                    }
                }
            }
            println!("Saved log to: {}", filename);
        }
        Err(e) => {
            eprintln!("Failed to create log file '{}': {}", filename, e);
        }
    }
}
