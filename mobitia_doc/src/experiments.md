# Experimental Procedure

この章では，論文執筆や性能評価のためのオフライン実験手順およびアンカー記録機能について説明します．

## 1. オフラインSLAM実験

Mobitiaには，過去の走行ログ（`scans.json`）を読み込んで異なるパラメータで再計算する「実験モード」が搭載されています．

### 基本コマンド
`--experiment` フラグを付けて実行することで，GUIを起動せずにヘッドレスモードで実験を実行します．

```bash
cargo run --release -- --experiment --mode [MODE] --input [LOG_DIR] --output [OUTPUT_DIR]
```

- **`--mode`**: 実験ケースを指定します．
    - `baseline`: 幾何特徴（エッジ・法線）を無効化した設定．
    - `proposed`: 推奨される標準設定．
    - `proposed_fast`: 計算量を1/4に削減した高速設定．
    - `proposed_eco`: 計算量を1/8に削減した省リソース設定．
    - `proposed_minimal`: 計算量を約1/10に削減した限界設定．
    - `ground_truth`: 探索範囲と個体数を大幅に増やした高精度な擬似真値生成設定（DEベース）．
    - `brute_force`: グリッドサーチによる全探索モード．確率的な要素を排し，設定された分解能内で確実に評価関数が最大となる姿勢を特定する．計算コストは非常に高い（1スキャン数秒〜）が，厳密な真値として機能する．
- **`--input`**: `slam_results/slam_result_YYYYMMDD...` などのログディレクトリを指定します．
- **`--output`**: 結果（CSV）の保存先ディレクトリを指定します．

### ヒートマップ出力 (Planned)
`brute_force` モードには，特定のタイムスタンプにおける評価関数の空間分布（ランドスケープ）をCSVとして出力する機能が追加される予定です．これにより，評価関数の形状や局所解の存在を可視化できます．

### 一括実行スクリプト
全パターンの実験と解析を一度に実行するためのシェルスクリプトが用意されています．

```bash
./scripts/run_all_experiments.sh [LOG_DIR]
```

このスクリプトを実行すると，`results/` ディレクトリ内に各モードの軌跡CSVが生成され，自動的に `scripts/analyze_results.py` が呼び出されて解析レポートとグラフが出力されます．

---

## 2. 実績解析ツール

実験で生成された複数の軌跡データを比較・評価するためのPythonスクリプトです．

```bash
python3 scripts/analyze_results.py --dir [RESULTS_ROOT_DIR]
```

### 生成される成果物
- **`experiment_summary.md`**: 平均並進誤差（APE），回転誤差，RMSEなどの統計レポート．
- **`trajectory_comparison.png`**: 各手法の軌跡をXY平面にプロットした図．
- **`error_comparison.png`**: Ground Truthに対する誤差の時系列プロット．
- **`error_decomposition.png`**: 誤差を「進行方向（Longitudinal）」と「横方向（Lateral）」に分解した解析図．

---

## 3. 静的アンカー記録機能 (Static Anchor Recording)

査読対応など，物理的な真値座標との比較を行うための機能です．

### 操作方法
SLAM実行中または自律走行中に，ロボットが特定の物理地点（アンカーポイント）に到達した際，以下のいずれかの操作を行います．

- **GUI**: 左側サイドパネルの **[⚓ Record Anchor (A)]** ボタンをクリック．
- **Shortcut**: キーボードの **`A`** キーを押下．

### 記録内容
プロジェクトのルートディレクトリに **`anchor_log.csv`** が作成（または追記）されます．

- **保存形式**: `timestamp, readable_time, x, y, theta, label`
- **活用方法**: ロボットをタイル目地などの既知の座標に静止させて記録し，後に実測値と比較することで位置推定の絶対精度を検証します．

---

## 4. アンカー比較ツール (Anchor Comparison Tool)

記録された `anchor_log.csv` と，オフライン実験で生成された軌跡データ（Trajectory）を照合し，位置推定の精度を定量評価するためのツールです．

### 概要
`scripts/compare_anchors.py` は，タイムスタンプに基づいてアンカー記録時点のロボットの推定位置（実験結果）を検索し，記録された物理的なアンカー座標（またはその時点でのリアルタイム推定値）との差異を計算します．

### 主な機能
- **自動フォーマット認識**: `anchor_log.csv` に含まれる余分な日時カラム（`readable_time`）の有無を自動検出し，適切にパースします．
- **タイムスタンプマッチング**: 指定された許容誤差（デフォルト15秒）以内で最も近い軌跡ポイントを探索します．
- **レポート生成**: 平均誤差などの統計情報を含むMarkdownレポートと，詳細データのCSVを出力します．

### 使用方法

```bash
python scripts/compare_anchors.py --anchors anchor_log.csv --trajectory results/brute_force/brute_force_trajectory.csv --output results --map_dir [SLAM_RESULT_DIR]
```

- **`--anchors`**: アンカーログファイルのパス．
- **`--trajectory`**: 比較対象の軌跡CSVファイル（`brute_force` モードの出力などを推奨）．
- **`--output`**: 結果（`anchor_comparison.md`, `anchor_comparison.csv`, `anchor_map_plot.png`）の保存先．
- **`--map_dir`**: (任意) `occMap.png` と `map_info.toml` が含まれるディレクトリを指定すると，アンカーをプロットした地図画像を生成します．

---

## 5. ランドスケープ（ヒートマップ）可視化ツール (Landscape Visualization Tool)

全探索（Brute-force）によって算出された評価関数の空間分布（コスト関数ランドスケープ）を可視化し，退化環境（Degenerate Environment）の解析を行うためのツールです．

### 概要
`scripts/plot_landscape.py` は，実験モードの `brute_force` 実行時に生成される `landscape_*.csv` を読み込み，2Dヒートマップおよび3Dサーフェスプロットを生成します．

### 主な機能
- **地図オーバーレイ**: `--map_dir` を指定することで，背景に構築済みの地図（`occMap.png`）を透過表示します．これにより，廊下などの環境形状と評価関数の「谷」の相関を視覚的に証明できます．
- **自動座標合わせ**: `map_info.toml` のメタデータを利用し，地図と評価関数の座標系を正確に一致させます．
- **3Dプロット**: 評価関数のピークの鋭さや局所解の存在を立体的に確認できます．

### 使用方法

#### 1. データの生成
まず，実験モードで `brute_force` を実行します．`--anchors` を指定すると，アンカー記録時刻に近いスキャンのランドスケープデータが自動的に出力されます．

```bash
cargo run --release -- --experiment --mode brute_force --input [LOG_DIR] --anchors [ANCHOR_LOG] --output [OUTPUT_DIR]
```

#### 2. 可視化の実行
出力された CSV ファイルを指定してプロットを生成します．

```bash
python scripts/plot_landscape.py [OUTPUT_DIR]/landscape_*.csv --output [PLOT_OUTPUT_DIR] --map_dir [LOG_DIR]
```

- **`--map_dir`**: `occMap.png` と `map_info.toml` が含まれるディレクトリを指定します．

<!-- TODO: Add English translation here -->
