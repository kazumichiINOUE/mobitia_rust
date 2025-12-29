use crate::app::LidarState;
use eframe::egui::Vec2;
use std::collections::HashMap;

// LiDARの初期設定を保持する構造体
#[derive(Clone, Debug)]
struct LidarConfigEntry {
    path: String,
    baud_rate: u32,
    origin: Vec2,
    rotation: f32,
    data_filter_angle_min: f32,
    data_filter_angle_max: f32,
}

pub fn load_lidar_configurations<F>(path_loader: F) -> Vec<LidarState>
where
    F: Fn(usize) -> Option<String>,
{
    // 2台のLidarの初期設定
    // TODO: 将来的には設定ファイルなどから読み込む
    let lidar_defs: HashMap<usize, LidarConfigEntry> = HashMap::from([
        (
            0, // 進行方向右手のlidar
            LidarConfigEntry {
                path: "/dev/cu.usbmodem1201".to_string(),
                baud_rate: 115200,
                origin: Vec2::new(0.0, -0.26),
                rotation: -std::f32::consts::FRAC_PI_2,
                data_filter_angle_min: -90.0f32,
                data_filter_angle_max: 135.0f32,
            },
        ),
        (
            1, // 進行方向左手のliar
            LidarConfigEntry {
                path: "/dev/cu.usbmodem1301".to_string(),
                baud_rate: 115200,
                origin: Vec2::new(0.0, 0.26),
                rotation: std::f32::consts::FRAC_PI_2 - std::f32::consts::PI * 1.0 / 180.0,
                data_filter_angle_min: -135.0f32,
                data_filter_angle_max: 90.0f32,
            },
        ),
        (
            2, // 予備のliar
            LidarConfigEntry {
                path: "/dev/cu.usbmodem2101".to_string(),
                baud_rate: 115200, // 正しい値に戻す
                origin: Vec2::new(0.0, 0.0),
                rotation: 0.0,
                data_filter_angle_min: -135.0f32,
                data_filter_angle_max: 135.0f32,
            },
        ),
    ]);
    let mut lidars = Vec::new();
    // HashMapをイテレートし、各LiDARの設定を取得
    for (&id, config_entry) in lidar_defs.iter() {
        // `path_loader` を使ってデバイスパスを読み込む
        let device_path = path_loader(id).unwrap_or_else(|| config_entry.path.clone());

        lidars.push(LidarState {
            id,
            path: device_path,
            baud_rate: config_entry.baud_rate,
            points: Vec::new(),
            connection_status: "Connecting...".to_string(),
            status_messages: Vec::new(),
            origin: config_entry.origin,
            rotation: config_entry.rotation,
            data_filter_angle_min: config_entry.data_filter_angle_min,
            data_filter_angle_max: config_entry.data_filter_angle_max,
            is_active_for_slam: id == 0 || id == 1,
        });
    }

    // IDでソートして、常に安定した順序を保証する
    lidars.sort_by_key(|l| l.id);
    lidars
}
