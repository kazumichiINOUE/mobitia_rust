### 2.10 非同期サブシステム: LiDAR (`src/lidar/`)
このモジュールは、LiDARセンサーとの通信、データ取得、および特徴量計算を担当します。`app`モジュールとは別のスレッドで動作し、取得したスキャンデータを`mpsc`チャネルを通じてメインスレッドに送信します。

モジュールは以下のファイルで構成されます。
- **`mod.rs`**: サブモジュールを宣言し、公開API (`start_lidar_thread`, `LidarInfo`) をエクスポートします。
- **`driver.rs`**: LiDARとの通信を行い、スキャンデータを取得するメインのドライバスレッドを実装します。
- **`comm.rs`**: シリアルポートのラッパーを提供し、低レベルの送受信を扱います。
- **`protocol.rs`**: LiDARの通信プロトコル（SCIP2.0）のデコード処理を実装します。
- **`features.rs`**: スキャンデータから特徴量（エッジ、法線、コーナー）を計算するロジックを実装します。

#### `driver.rs` (LiDARドライバ)
`start_lidar_thread` 関数がこのモジュールのエントリーポイントです。

1.  **初期化**:
    - `LidarDriver::new` でシリアルポートの存在を確認し、`comm::LidarConnection` を開きます。
    - `LidarDriver::initialize` でLiDARに初期化コマンド（`VV`, `PP`, `II`, `BM`）を送信し、レーザーをオンにします。
2.  **データ取得ループ**:
    - 無限ループの中で、100msごとに `LidarDriver::get_distance_data` を呼び出します。
    - `get_distance_data` は、`GD`コマンドをLiDARに送信し、3文字または4文字でエンコードされた距離データを受信します。
    - `protocol::decode_scip_2_0_3char` を使って距離データをデコードし、各ステップの角度と組み合わせて点群データ `(x, y, r, theta, ...)` を生成します。
    - 生成された点群は `LidarMessage::ScanUpdate` メッセージにラップされ、`mpsc`チャネルを通じて `MyApp` に送信されます。

```rust
// start_lidar_thread の抜粋
pub fn start_lidar_thread(
    lidar_id: usize,
    lidar_config: LidarInfo,
    message_sender: mpsc::Sender<LidarMessage>,
) {
    thread::spawn(move || {
        // ... (LidarDriver::new, driver.initialize) ...
        
        loop {
            match driver.get_distance_data() {
                Ok(points) => {
                    message_sender.send(LidarMessage::ScanUpdate {
                        id: lidar_id,
                        scan: points,
                    });
                }
                Err(e) => { /* ... エラー処理 ... */ }
            }
            thread::sleep(Duration::from_millis(100));
        }
    });
}
```

#### `features.rs` (特徴量計算)
スキャンデータに付加情報を与え、SLAMの精度を向上させるための特徴量を計算します。`MyApp::update` 内の `LidarMessage::ScanUpdate` 受信時に呼び出されます。

- **`compute_features`**:
    1.  **エッジと法線の計算**: 各点の近傍点（左右5点ずつ）の分布から共分散行列を計算し、その固有値・固有ベクトルを求めます。
        - 固有値の比から直線性（linearity）を計算し、それをシグモイド関数に通すことで**エッジらしさ (`edge_ness`)** を算出します。
        - 小さい方の固有値に対応する固有ベクトルを、その点の**法線ベクトル (`normal_vector`)** とします。
    2.  **コーナーの計算**:
        - 各点の前後数点の法線ベクトルの平均を比較し、その内積から法線の変化量を求め、**コーナーらしさ (`corner_ness`)** を計算します。点が密な領域でのみ計算されます。
        - また、隣接点との距離が急に大きくなる「ステップエッジ」もコーナーとして検出し、`corner_ness` を更新します。
- **`interpolate_lidar_scan`**:
    - SLAMのマッチング精度を向上させるため、スキャン点群の間隔が広い部分を線形補間して、より密な点群を生成します。
    - `min_dist_threshold` 以下の近すぎる点を除去（thinning）した後、`interpolation_interval` に基づいて点を追加します。

```rust
// compute_features の抜粋
pub fn compute_features(
    scan: &Vec< /* ... */ >,
) -> Vec< /* ... */ > {
    // ...
    for i in neighborhood_size..(scan.len() - neighborhood_size) {
        // ... (共分散行列と固有値計算) ...
        let linearity = (lambda_1 - lambda_2) / lambda_1;
        let edge_ness = 1.0 - (1.0 / (1.0 + (-sharpness * (linearity - sensitivity)).exp()));

        scan_with_features[i].4 = edge_ness;
        scan_with_features[i].5 = corrected_nx;
        scan_with_features[i].6 = corrected_ny;
    }
    // ...
    for i in neighborhood_size..(scan.len() - neighborhood_size) {
        // ... (法線ベクトルの比較によるコーナー計算) ...
        let dot_product_normals = normal_before.dot(&normal_after);
        corner_ness = 1.0 - dot_product_normals.abs();
        scan_with_features[i].7 = corner_ness;
    }
    // ...
    scan_with_features
}
```