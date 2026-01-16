### 2.7 非同期サブシステム: モーター制御 (`src/motors/mod.rs`)
このモジュールは、モータードライバとのMODBUSシリアル通信、コマンド実行、およびオドメトリ計算を担当します。`start_modbus_motor_thread` によって起動される単一の管理スレッドが、内部でさらにオドメトリ計算用の高頻度スレッドを起動する構成になっています。

#### 関連する型定義

```rust
// UIスレッドからモーター制御スレッドへ送信されるコマンド
#[derive(Debug, Clone, Copy)]
pub enum MotorCommand {
    SetVelocity(f32, f32), // v (m/s), w (rad/s)
    SetVelocityTimed(f32, f32, u64), // v, w, duration (ms)
    Stop,
    EnableIdShare, // 左右モーターの同期モード有効化
    ServoOn,
    ServoOff,
    ServoFree,
    ReadState, // モーターの詳細状態を要求
}

// モーター制御スレッドからUIスレッドへ送信されるメッセージ
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub alarm_code_r: i32,
    pub temp_driver_r: f32,
    pub temp_motor_r: f32,
    pub position_r: i32, // 右モーターのエンコーダ値
    pub power_r: i32,
    pub voltage_r: f32,
    pub alarm_code_l: i32,
    pub temp_driver_l: f32,
    pub temp_motor_l: f32,
    pub position_l: i32, // 左モーターのエンコーダ値
    pub power_l: i32,
    pub voltage_l: f32,
}

#[derive(Debug)]
pub enum MotorMessage {
    Status(String),
    StateUpdate(MotorState),
}

// オドメトリ計算スレッドの内部状態
#[derive(Debug, Default, Clone)]
struct OdoState {
    initialized: bool,
    last_pos_r: i32,
    last_pos_l: i32,
    x: f32,
    y: f32,
    angle: f32, // radians
}
```

#### MODBUS通信
- **コマンドテンプレート**: `QUERY_NET_ID_WRITE_TEMPLATE` や `QUERY_IDSHARE_R` などの `const` 配列として、頻繁に使用するMODBUSコマンドのテンプレートが定義されています。
- **CRC計算**: `calculate_crc16_modbus` 関数が、`crc` クレートを使用してMODBUSコマンドに必要なCRCチェックサムを計算します。
- **送受信**: `send_and_read` 関数が、コマンドへのCRC付与、シリアルポートへの書き込み、およびレスポンスの読み取りをカプセル化します。

#### `start_modbus_motor_thread`
この関数がモジュール全体の動作を開始します。

1.  `config.motor.port` で指定されたシリアルポートを開きます。
2.  オドメトリデータを `MyApp` と共有するための `Arc<Mutex<(f32, f32, f32)>>` を生成します。
3.  メインの管理スレッドを `thread::spawn` で起動します。
    - このスレッド内で、さらに**オドメトリ計算スレッド**を `thread::spawn` で起動します。
    - その後、**コマンド処理ループ**に入ります。
4.  管理スレッドのハンドルと、共有オドメトリへの参照を返します。

##### オドメトリ計算スレッド
- **周期**: 50msのループ。
- **処理**:
    1. `QUERY_NET_ID_READ` コマンドを送信し、左右モーターのエンコーダ値 (`position_l`, `position_r`) を一括で取得します。
    2. `circular_diff32` を用いて、前回取得した値との差分を計算します。これによりエンコーダ値のオーバーフローに対応します。
    3. `config.toml` から取得したロボットの物理パラメータ（車輪径、トレッド幅など）を用いて、エンコーダ差分をロボットの移動量 `dl` (直進距離) と `dth` (回転角度) に変換します。
    4. `OdoState` に積算されているロボットのグローバル座標 `(x, y, angle)` を更新します。
    5. 更新された座標を `shared_odometry` (`Arc<Mutex<...>>`) に書き込みます。

```rust
// Odometry Thread Logic Snippet
thread::spawn(move || {
    let mut odo_state = OdoState::default();
    loop {
        thread::sleep(Duration::from_millis(50));
        match send_and_read(&odom_port, &mut QUERY_NET_ID_READ.to_vec(), 57) {
            Ok(buf) => {
                let current_pos_r = get_i32(&buf[15..19]);
                let current_pos_l = get_i32(&buf[15 + OFFSET..19 + OFFSET]);

                // ... (delta_pos_r, delta_pos_l calculation) ...

                let dist_l = (delta_pos_l as f32) * step_res_rad * 0.5 * wheel_d / gear_ratio;
                let dist_r = -(delta_pos_r as f32) * step_res_rad * 0.5 * wheel_d / gear_ratio;

                let dl = (dist_l + dist_r) / 2.0;
                let dth = (dist_r - dist_l) / wheel_t;

                odo_state.x += dl * odo_state.angle.cos();
                odo_state.y += dl * odo_state.angle.sin();
                odo_state.angle += dth;

                // ... (update last positions and shared_odometry) ...
            }
            Err(_) => { /* ... */ }
        }
    }
});
```

##### コマンド処理ループ
- `mpsc` チャネル `command_receiver` を `try_recv` でポーリングします。
- `MotorCommand` を受信すると、`match` 文で分岐し、対応する処理を実行します。
- `SetVelocity` や `Stop` の場合、`calc_vw_to_motor_rpms` で `v` と `w` をモーターのRPMに変換し、`QUERY_NET_ID_WRITE_TEMPLATE` を使ってコマンドを送信します。
- `EnableIdShare` のような初期化コマンドは、定義済みのバイト列を順次送信します。
