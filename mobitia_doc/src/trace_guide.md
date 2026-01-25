# 4.1 システムトレーシング (Perfetto)
<!-- TODO: Add English translation here -->

Mobitiaでは、`tracing` クレートを使用して、ミリ秒単位のイベントや関数の実行時間を記録しています。これにより、「なぜか止まらない」「挙動が不安定になる」といった再現性の低い問題の原因究明が容易になります。

## ログファイル
アプリケーションを実行すると、自動的に以下の場所にトレースログが保存されます。
- **場所**: `./traces/trace.json`
- **形式**: Chrome Trace Event Format (JSON)

## 可視化ツール: Perfetto
<!-- TODO: Add English translation here -->

生成された JSON ファイルは、そのままでは人間が読むのは困難です。Google が提供する **[Perfetto UI](https://ui.perfetto.dev/)** を使用して可視化します。

### 使用手順
1. ブラウザで [https://ui.perfetto.dev/](https://ui.perfetto.dev/) を開きます。
2. 左メニューの `Open trace file` をクリックし、`traces/trace.json` をアップロードします。
3. タイムラインが表示されます。

## 解析のポイント
<!-- TODO: Add English translation here -->

### 1. スレッドごとの動き
- `main` (UIスレッド): `update` 関数の実行時間やフレームレートが確認できます。
- `mobitia::motors`: モーター制御指令の送受信タイミングが確認できます。

### 2. 主要なマーカー (Events)
検索ボックス (右上) を使用して、以下のキーワードで重要なイベントを特定できます。
- **`Localization Converged`**: 自己位置推定が完了した瞬間。
- **`GOAL REACHED`**: ゴール判定が下された瞬間。
- **`COLLISION AVOIDANCE`**: LiDARによる緊急回避が発動した瞬間。

これらのイベントをクリックすると、タイムライン上の正確な位置にジャンプできます。

### 3. 関数の実行時間
各バー（スパン）をクリックすると、下部の詳細パネルにその関数の実行時間 (Duration) が表示されます。`ElasticBand::optimize` など、負荷の高い処理がフレームタイムを圧迫していないか確認できます。

## コードへの計装方法
<!-- TODO: Add English translation here -->

新しい処理を追跡対象にしたい場合は、関数に `#[tracing::instrument]` アノテーションを付けるか、重要なタイミングで `tracing::info!("メッセージ")` を記述します。

```rust
#[tracing::instrument]
fn my_complex_logic() {
    // ... 処理 ...
    tracing::info!("重要な分岐を通過");
}
```