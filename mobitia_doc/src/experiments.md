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
- **SLAM Continuous実行中**: 各セッションの出力ディレクトリ（`slam_results/slam_result_.../`）内に **`anchor_log.csv`** が作成されます．
- **それ以外（Manual等）**: プロジェクトのルートディレクトリに作成・追記されます．

**保存形式**: `timestamp, readable_time, x, y, theta, label, x_true, y_true, theta_true_deg`

- **活用方法**: ロボットをタイル目地などの既知の座標に静止させて記録し，後に実測値を記入することで，物理真値に基づいた正確な軌跡アライメント（第8章参照）に利用します．

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
- **`--output`**: 結果（`anchor_comparison.md`, `anchor_comparison.csv`, `anchor_map_plot.pdf`）の保存先．
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

---

## 6. Brute-force解析の高速化 (Optimization)

Brute-forceモードは計算負荷が非常に高いですが，`rayon` ライブラリによる並列化と間引きオプションの実装により，実用的な速度での解析が可能になりました．

### 並列化 (Parallelization)
空間探索のメインループ（角度探索）を並列化することで，CPUの全コアを活用します．
- **効果**: Apple M3 Max (14/16コア) などのメニーコア環境では，シングルスレッド比で **10倍以上** の高速化（CPU使用率1400%超）が期待できます．
- **自動適用**: `Cargo.toml` に `rayon` が含まれていれば，自動的に全コアを使用して計算を行います．

### ステップ実行 (Temporal Subsampling)
予備実験や統計解析のために，スキャンデータを間引いて処理することが可能です．

```bash
cargo run --release -- --experiment --mode brute_force ... --step 5
```

- **`--step N`**: $N$ 回に1回のみ解析を実行します（デフォルトは1=全フレーム解析）．
- **用途**: 長時間のログから「退化エリアの傾向」だけを素早く把握したい場合に，`--step 10` などで実行時間を1/10に短縮できます．

---

## 7. 統計的退化解析 (Statistical Degeneracy Analysis)

環境の幾何学的退化（Degeneracy）はノイズや経路の微細な変化に敏感であるため，単一の走行データではなく，複数回の走行データを統合して統計的に評価エリアを特定する手法を導入しました．

### 概要
`scripts/analyze_multi_run_degeneracy.py` は，複数の `slam_result` ディレクトリを一括処理し，以下のプロセスを実行します．

1.  **Brute-force解析**: 未解析のログに対して自動的に全探索解析を実行します．
2.  **軌跡アライメント (Trajectory Alignment)**: 基準となる軌跡（データ点数最大）に対し，他の軌跡を **ICP (Iterative Closest Point)** アルゴリズムを用いて精密に位置合わせします．これにより，異なる走行間の座標ズレを補正します．
3.  **統計マップ生成**: 空間グリッド（デフォルト5cm）ごとに最小固有値の平均を計算し，ヒートマップとして可視化します．
4.  **領域抽出と楕円フィッティング**:
    - 退化（Degenerate）および安定（Stable）の条件を満たす連結領域を **BFS (幅優先探索)** で抽出します．
    - データのスパース性を補うため，モルフォロジー演算（Dilation）による穴埋め処理を適用します．
    - 抽出された点群の共分散行列から **楕円 (Ellipse)** を算出し，領域の広がりと方向を可視化します．

### 使用方法

1.  ルートディレクトリに解析用フォルダ（例：`experiment_degeneracy_stats`）を作成し，対象となる複数の `slam_result_XXX` フォルダを格納します．
2.  スクリプトを実行します．`--grid_size` は解像度に合わせて調整します（推奨: 0.05）．

```bash
python3 scripts/analyze_multi_run_degeneracy.py experiment_degeneracy_stats --output multi_run_results --grid_size 0.05
```

### 成果物
- **`multi_run_degeneracy_map.pdf`**: 統計的退化マップ上に，抽出された検証ポイント（D1-D5, S1-S5）とその有効領域（楕円）がプロットされた地図．
- **`final_verification_points.csv`**: 物理計測のターゲットとなる座標と，領域の形状パラメータ（長軸，短軸，角度）のリスト．

---

## 8. 物理真値に基づく統合アライメント解析 (Integrated Alignment Analysis)

「最初と最後」の2点の物理真値（Ground Truth）のみを拘束条件として使用し，恣意的なパラメータ調整を排した状態で全体の軌跡と地図を幾何学的に補正・集計する一貫フローです．

### 概要
`scripts/run_anchor_aligned_analysis.sh` を実行することで，以下の4段階の処理が自動的に行われます．

1.  **Stage 1: 基礎解析 (Base SLAM Analysis)**:
    - 各生のログに対して `brute_force`（全探索）を実行し，基準となる退化指数（固有値等）を算出します．既存の解析結果がある場合はスキップされます．
2.  **Stage 2: 幾何学的アライメント (Geometric Alignment)**:
    - 各ディレクトリの `anchor_log.csv` に入力された `x_true, y_true, theta_true_deg` を参照し，軌跡全体および退化ログの座標を剛体変換（回転・並進）します．Pythonで計算するため瞬時に完了します．
3.  **Stage 3: 補正地図の再構築 (Map Reconstruction)**:
    - 補正された座標を「正解」として，`brute_force_mapping_only` モードで占有格子地図（`occMap.png`）を描き直します．全探索を行わないため非常に高速です．
4.  **Stage 4: 統計的統合 (Statistical Consolidation)**:
    - 全てが補正された状態で `analyze_multi_run_degeneracy.py` を実行し，最終的な統計マップと検証ポイントを抽出します．

### 使用方法

1.  対象とする複数の `slam_result_...` ディレクトリを一つのフォルダ（例: `experiment_anchor_test`）に集めます．
2.  各ディレクトリ内の `anchor_log.csv` を開き，実測した物理座標を `x_true, y_true, theta_true_deg` に記入します．
3.  スクリプトを実行します．

```bash
./scripts/run_anchor_aligned_analysis.sh
```

### メリット
- **客観性の担保**: 物理真値という確定した事実のみに基づいてアライメントが行われます．
- **計算効率**: 重い全探索（Brute-force）は各ログにつき一度しか実行されません．
- **地図の正当性**: 生成されるPDFの背景地図そのものが物理的な方位・位置に合わせて回転・補正されます．

<!-- TODO: Add English translation here -->