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

<!-- TODO: Add English translation here -->
