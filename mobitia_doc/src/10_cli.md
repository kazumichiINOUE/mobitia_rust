### 2.11 コマンドラインインターフェース (`src/cli.rs`)
このモジュールは、フローティングコンソールから入力されるコマンドの定義と、その処理ロジックを担います。`clap` クレートの `derive` マクロを多用し、構造体や `enum` の定義からコマンドパーサーを自動生成しています。

#### コマンドの構造 (`Cli` と `Commands`)
- **`Cli`**: 全てのコマンドのルートとなる構造体です。
- **`Commands`**: 利用可能な全てのサブコマンドを定義した `enum` です。各バリアントが `help`, `lidar`, `slam`, `motor` などのトップレベルコマンドに対応します。
- サブコマンドはさらにネストした `enum`（例: `MotorCommands`, `LidarCommands`）で定義され、階層的なコマンド構造を実現しています。

```rust
/// CLI Commands for Mobitia application
#[derive(Parser, Debug)]
#[command(/* ... clap attributes ... */)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Show help for commands.
    #[command(alias = "h")]
    Help,
    
    /// Manage motor controls.
    Motor {
        #[command(subcommand)]
        command: MotorCommands,
    },
    
    /// Manage LiDAR related commands and settings.
    Lidar {
        #[command(subcommand)]
        command: Option<LidarCommands>,
    },
    
    /// Enter SLAM mode.
    Slam {
        #[command(subcommand)]
        command: Option<SlamCommands>,
    },
    
    // ... 他のコマンド ...
}

#[derive(Subcommand, Debug)]
pub enum MotorCommands {
    /// Set velocity (m/s) and omega (rad/s).
    Set {
        #[arg(short, long)]
        velocity: f32,
        #[arg(short, long)]
        omega: f32,
    },
    // ... 他のモーターサブコマンド ...
    Stop,
    EnableIdShare,
}
```

#### コマンド処理 (`handle_command` 関数)
- **エントリーポイント**: `app.rs` の `update` メソッド内、コンソールで `Enter` が押された際に呼び出されます。
- **処理の流れ**:
    1.  `shlex::split` でユーザーの入力文字列をシェルライクに分割します。
    2.  `Cli::try_parse_from` で分割された文字列をパースし、`Cli` 構造体のインスタンスを生成します。パースに失敗した場合は、`clap` が生成したエラーメッセージがコンソールに表示されます。
    3.  パースに成功すると、`handle_command` の `match cli.command` ブロックに処理が移ります。
    4.  各 `Commands` バリアントに対応する `match` アームが、`MyApp` の状態を変更したり、各サブシステムのスレッドにコマンドを送信したりします。

##### 処理の例
- **`Commands::Motor { command }`**:
    - 受信した `MotorCommands` (`Set`, `Stop` など) に応じて、`app.motor_command_sender` を通じてモーター制御スレッドに `crate::motors::MotorCommand` を送信します。
- **`Commands::Slam { command }`**:
    - アプリケーションのモードを `AppMode::Slam` に変更します。
    - `SlamCommands` (`Continuous`, `Pause` など) に応じて、`app.slam_command_sender` を通じてSLAMスレッドに `crate::app::SlamThreadCommand` を送信します。
- **`Commands::Help`**:
    - `clap` の機能を利用して、定義されたコマンド構造からヘルプメッセージを自動生成し、コンソールに表示します。

```rust
// handle_command の抜粋
pub fn handle_command(app: &mut MyApp, ctx: &egui::Context, cli: Cli) {
    match cli.command {
        Commands::Help => {
            let help_text = Cli::command().render_help().to_string();
            // ... ヘルプテキストをコンソールに出力 ...
        }
        Commands::Motor { command } => {
            let send_motor_cmd = |app: &mut MyApp, cmd: crate::motors::MotorCommand| {
                app.motor_command_sender.send(cmd);
            };

            match command {
                MotorCommands::Set { velocity, omega } => {
                    send_motor_cmd(app, crate::motors::MotorCommand::SetVelocity(velocity, omega));
                }
                // ...
            }
        }
        Commands::Slam { command } => {
            app.app_mode = AppMode::Slam;
            if let Some(slam_command) = command {
                match slam_command {
                    SlamCommands::Continuous => {
                        app.slam_mode = crate::app::SlamMode::Continuous;
                        app.slam_command_sender.send(crate::app::SlamThreadCommand::StartContinuous);
                    }
                    // ...
                }
            }
        }
        // ... 他のコマンドハンドリング ...
    }
}
```