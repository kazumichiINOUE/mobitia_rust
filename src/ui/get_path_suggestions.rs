use std::fs;
use std::path::{Path, PathBuf};

/// Helper function for path suggestions.
/// Lists entries in `dir_path` that start with `prefix`.
pub fn get_path_suggestions(dir_path: &str, prefix: &str) -> Vec<String> {
    let mut suggestions = Vec::new();
    let dir_to_read = if dir_path.is_empty() {
        Path::new(".")
    } else {
        Path::new(dir_path)
    };

    if let Ok(entries) = fs::read_dir(dir_to_read) {
        for entry in entries.filter_map(|e| e.ok()) {
            if let Some(file_name_os) = entry.file_name().to_str() {
                if file_name_os.starts_with(prefix) {
                    let mut suggestion_str = file_name_os.to_string();
                    if entry.path().is_dir() {
                        suggestion_str.push('/');
                    }
                    suggestions.push(suggestion_str);
                }
            }
        }
    }
    suggestions.sort();
    suggestions
}
