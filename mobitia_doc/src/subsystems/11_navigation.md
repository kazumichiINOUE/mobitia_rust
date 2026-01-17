# 2.11 非同期サブシステム: ナビゲーション (`src/navigation/`)
このモジュールは、ロボットの経路計画と追従、自己位置推定など、ナビゲーションに関する機能を提供します。

## ナビゲーションモード (Nav Mode)
`nav` コマンドが実行されると、アプリケーションはナビゲーションモードに切り替わります。このモードは、ナビゲーションに関連する全ての機能（テスト実行、ナビゲーション開始など）の結果や状態を視覚的に表示するための専用描画モードです。地図、ロボットの自己位置、およびLiDAR計測データがリアルタイムに描画されます。

## コマンド
`nav` コマンドは、ナビゲーション機能の制御を行います。

### `nav test <path>`
指定されたパス（例: マップファイル、経路ファイル）を使用してナビゲーションテストを実行します。テストの進行状況や結果はナビゲーションモードで視覚化されます。

### `nav start <path>`
指定されたパスの経路を読み込み、ナビゲーションを開始します。ロボットの移動や経路追従の様子はナビゲーションモードで視覚化されます。

### 共通の初期処理と表示
`nav test <path>` および `nav start <path>` コマンドの実行時には、`<path>` で指定されたディレクトリから以下のファイルが読み込まれます。
- `occMap.png`: 占有格子地図の画像ファイル。
- `map_info.toml`: 地図情報（解像度、原点など）を記述した設定ファイル。
- `trajectory.txt`: ナビゲーションのターゲットポイントのXY座標（1列目と2列目）が記述されたファイル。

読み込まれた `occMap.png` は画面全体に表示されます。また、`map_info.toml` の情報に基づいて地図の原点とXY軸が目立たない色で描画されます。`trajectory.txt` から読み取られたターゲットポイントも、同様に目立たない色で地図上に表示されます。

### `nav stop`
現在実行中のナビゲーションプロセスを停止します。このコマンドの実行後、アプリケーションの描画モードはデフォルトでLiDARモードに戻ります。

## アーキテクチャと設計方針

### モジュール構成
ナビゲーション機能は `src/navigation/` ディレクトリ以下のモジュールに集約されています。

- **`src/navigation/mod.rs`**: `NavigationManager` 構造体とその実装が含まれます。これがナビゲーションサブシステムのメインエントリーポイントです。
- **`src/navigation/localization.rs`**: 将来的な自己位置推定ロジック（現状は定義のみ）。
- **`src/navigation/pure_pursuit.rs`**: 将来的な経路追従ロジック（現状は定義のみ）。

### `NavigationManager`
`src/app.rs` の肥大化を防ぐため、ナビゲーションに関連する状態とロジックは `NavigationManager` にカプセル化されています。`MyApp` はこのマネージャーのインスタンスを保持し、処理を委譲します。

#### 管理する状態
- **地図データ**: `nav_map_texture`, `nav_map_bounds`
- **経路情報**: `nav_trajectory_points`
- **現在のターゲット**: `current_nav_target`
- **ロボットの現在姿勢**: `current_robot_pose`
- **オドメトリ履歴**: `last_odom`（前回フレームの値を保持し、差分計算に使用）

### 自己位置更新ロジック
`NavigationManager::update` メソッドは毎フレーム呼び出され、以下の手順でロボットの自己位置を更新します。この設計は、将来的にLiDARスキャンマッチングなどの他のセンサー情報を融合させる際の拡張性を考慮しています。

1.  **オドメトリ差分の計算**: モーター制御スレッドから得られる現在のオドメトリ値と、前回フレームの値 (`last_odom`) との差分を計算します。
2.  **座標変換**: グローバル座標系での差分を、ロボットのローカル座標系での移動量に変換します。
3.  **姿勢の更新**: 変換された移動量を現在のロボット姿勢 (`current_robot_pose`) に適用します。

```rust
// src/navigation/mod.rs の update メソッド（抜粋）
pub fn update(&mut self, current_odom: (f32, f32, f32)) {
    if let Some((last_x, last_y, last_theta)) = self.last_odom {
        // ... (差分計算と座標変換) ...
        
        let movement = Isometry2::new(
            nalgebra::Vector2::new(delta_x_local, delta_y_local),
            delta_theta,
        );
        self.current_robot_pose *= movement;
    }
    self.last_odom = Some(current_odom);
}
```

## サジェスト
`nav` コマンドとそのサブコマンド (`test`, `start`, `stop`)、および `<path>` 引数は、コマンド入力時のサジェストの対象となります。
