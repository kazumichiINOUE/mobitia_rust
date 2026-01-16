### 2.8 非同期サブシステム: SLAM (`src/slam/mod.rs`)
このモジュールは、LiDARスキャンとオドメトリ情報を受け取り、ロボットの自己位置推定と環境地図の生成（SLAM）を行う中心的な役割を担います。`app`モジュールとは別のスレッドで動作し、重い計算処理をUIスレッドから分離します。

#### 関連する型定義

```rust
// 占有格子地図の各セルの情報
#[derive(Clone, Copy, Debug)]
pub struct CellData {
    pub log_odds: f64,          // 占有確率（対数オッズ）
    pub edge_ness: f64,         // エッジらしさ (0.0: 直線, 1.0: エッジ)
    pub normal_x: f64,          // 法線ベクトルX
    pub normal_y: f64,          // 法線ベクトルY
    pub centroid_x: f64,        // そのセルに含まれる点の重心X
    pub centroid_y: f64,        // そのセルに含まれる点の重心Y
    pub point_count: u32,       // そのセルに含まれた点の数
    pub corner_ness: f64,       // コーナーらしさ
}

// 占有格子地図
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub data: Vec<CellData>,
}

// サブマップのメタデータ
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Submap {
    pub id: usize,
    #[serde(skip)]
    pub global_pose: Isometry2<f32>, // サブマップ生成開始時のグローバル姿勢
    pub pose_x: f32,
    pub pose_y: f32,
    pub pose_theta: f32,
    pub timestamp_ms: u128,
    pub scans_file: String, // サブマップを構成するスキャンデータファイル名
    pub info_file: String,  // このメタデータ自身のファイル名
}

// ファイル保存用のスキャンデータ
#[derive(Serialize, Deserialize)]
pub struct ScanData {
    pub timestamp: u128,
    pub relative_pose: Pose, // サブマップ基準の相対姿勢
    pub scan_points: Vec<ScanPoint>,
}
// ... (Pose, ScanPoint は省略) ...
```

#### `SlamManager` 構造体
SLAM処理全体の状態を管理します。

```rust
pub struct SlamManager {
    is_initial_scan: bool,
    map_gmap: OccupancyGrid, // メインの占有格子地図
    robot_pose: Isometry2<f32>, // 現在のロボットの姿勢
    de_solver: differential_evolution::DifferentialEvolutionSolver, // 姿勢推定のための最適化ソルバー
    pub(crate) config: SlamConfig,
    log_odds_occ: f64,  // 占有を示すlog-odds値
    log_odds_free: f64, // 空きを示すlog-odds値

    // サブマップ関連
    submap_counter: usize,
    current_submap_scan_buffer: Vec<Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>>,
    current_submap_robot_poses: Vec<Isometry2<f32>>,
    // ...

    // UI描画用のキャッシュ
    cached_map_points: Vec<(Point2<f32>, f64)>,
    is_map_dirty: bool,
}
```

#### 主要なロジック

##### `SlamManager::new`
- `SlamConfig` に基づいて、`OccupancyGrid` と差分進化ソルバー `de_solver` を初期化します。
- `prob_occupied` と `prob_free` から、地図更新に使用する `log_odds` の値を計算しておきます。

##### `SlamManager::update`
SLAM処理のメインステップです。UIスレッドから `SlamThreadCommand` を通じて呼び出されます。

1.  **地図の減衰 (Map Decay)**: `config.decay_rate` に基づき、地図全体の `log_odds` を少しずつ0（未知）に近づけます。これにより、古い情報の影響を徐々に減らします。
2.  **姿勢推定 (Pose Estimation)**:
    - `de_solver.optimize_de` を呼び出します。
    - この関数は、現在のスキャンデータ (`matching_scan`) と地図 (`map_gmap`) が最もよく一致するようなロボットの姿勢 `(x, y, angle)` を、差分進化アルゴリズムを用いて探索します。
    - 最適化の結果得られた `best_pose` を新しい `self.robot_pose` として更新します。
    - オプションとして渡された `odom_guess` を、探索の初期推測値として利用し、最適化を高速化・安定化させます。`odom_guess` は、`config.toml` の `use_odometry_as_initial_guess` が `true` の場合に `app.rs` 内で計算され、そのプロセスは以下の通りです。
        1.  **オドメトリの取得**: モーター制御スレッドは、常にロボットのグローバル座標系での積算位置（オドメトリ）を計算し、`MyApp` はこれを `motor_odometry`として保持しています。
        2.  **差分の計算**: SLAMの更新タイミングで、現在の `motor_odometry` と、**前回のSLAM更新時に保存しておいたオドメトリ** (`last_slam_odom`) との差分を計算します。これにより、グローバル座標系でのロボットの移動量 `(delta_x_global, delta_y_global, delta_angle)` が得られます。
        3.  **座標系の変換**: グローバル座標系での移動量を、**前回のSLAM更新時の姿勢** (`last_slam_odom_angle`) を使って、ロボットのローカル座標系での移動量 `(dx, dy)` に変換します。これは、SLAMの最適化ソルバーが、ロボット自身の座標系での「前進/後退」と「左右移動」を推測値として期待するためです。
        4.  **初期推測値の提供**: このようにして計算されたローカル座標系での移動量 `(dx, dy)` と角度の変化量 `delta_angle` を組み合わせたタプルが、`odom_guess` としてSLAMの最適化ソルバーに渡されます。
3.  **地図更新 (Map Update)**:
    - 新しく確定したロボット姿勢 `robot_pose` を使って、現在のスキャンデータを地図に反映させます。
    - `update_grid_probabilistic` メソッドが呼ばれます。

```rust
// SlamManager::update の抜粋
pub fn update(
    &mut self,
    // ... (raw_scan_data, interpolated_scan_data, etc.)
    odom_guess: Option<(f32, f32, f32)>,
) {
    // 1. 地図の減衰
    // ...

    // 2. 姿勢推定
    if self.is_initial_scan {
        // ...
    } else {
        let (best_pose, _score) = self.de_solver.optimize_de(
            &self.map_gmap,
            &matching_scan,
            &raw_corner_points,
            self.robot_pose,
            odom_guess,
        );
        self.robot_pose = best_pose;
    }

    // 3. 地図更新
    match self.config.map_update_method {
        MapUpdateMethod::Probabilistic | MapUpdateMethod::Hybrid => {
            self.update_grid_probabilistic(&mapping_scan, &mapping_scan_with_features, &self.robot_pose);
        }
        // ...
    }
    self.is_map_dirty = true;
}
```

##### `update_grid_probabilistic`
確率的な占有格子地図の更新ロジックです。逆センサーモデルに基づいています。

1.  **`update_free_space`**: センサーの原点からスキャン点までの光線が通過した領域を「空き領域」として更新します。
    - 各光線について、Bresenham's algorithm を用いて光線上のセルを特定します。
    - これらのセルの `log_odds` を `log_odds_free` だけ減少させます。
2.  **`update_occupied_space`**: スキャン点が実際に当たったセルを「占有領域」として更新します。
    - `log_odds` を `log_odds_occ` だけ増加させます。
    - 同時に、そのセルの特徴量（`edge_ness`, `corner_ness`）や法線ベクトルを、スキャン点の特徴量と加重平均で更新します。
