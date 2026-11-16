# 装填（ST3215 ステップモード）の制御 設計書（gamma、2026-11-16）

トリガーホイール式の装填機構（設計メモ `Rabbits/docs/superpowers/specs/2026-07-26-bb-feeder-trigger-wheel-design.md`）を、PC の `load` モードから ST3215 のステップ指令で回す。rabcl に `Feeder`（いつ何カウント送るかを決める、機体の値なし）、gamma_turret_fw に `FeederTask`（サーボと話す FreeRTOS タスク）を足す。

## 背景

- 単体試験（2026-10-31、rabcl `St3215` + 試験モード）で、ステップモードは相対カウントが積算され、多回転でき、負方向も同じ、所要時間は speed / acc で決まる（171 カウント: 3000 / 254 で到達 160 ms・行き過ぎ 3、acc 0 で 60 ms・行き過ぎ 22）ことが分かっている。
- メカは増速 3:1（装填側 = サーボの 3 倍）、ホイール 8 セル → サーボ 1 セル = 4096 ÷ 8 ÷ 3 = **170.67 カウント**（非整数。固定値を送り続けると約 300 発で穴とずれる → 実数で累積して round の差分を送る）。
- ESC の実装と同じく、`Controller::Tick` が PC の指令を解釈し、ハードに書くのは別のところ（ここでは専用タスク）。

## 決めたこと

| 項目 | 決定 |
|---|---|
| `load = 1`（L1） | 押した瞬間に 1 セル、押し続ければ `FEED_PERIOD_MS` ごとに 1 セル（連射）。短く叩けば 1 発。離してすぐ押し直したときは、前の発から周期が経っていればすぐ、まだなら周期になった時点で 1 発（押下は取りこぼさない、チャタリングで 2 発出ない） |
| `load = 2`（○） | 逆転。同じ周期で 1 セルずつ戻す（ジャム復旧用）。速度は `FEED_REVERSE_SPEED` |
| `load = 0`、MODE 未受信 | 止まる（送り済みのステップは完了する）。押下エッジで保留した 1 発は出す |
| 途絶（`WATCHDOG_S`） | 止まる。保留も捨てる（`Feeder::Cancel`） |
| 周期・速度 | **100 ms / speed 3000 / acc 0（秒 10 発）**。2026-11-16 にサーボ単体で 200 ms / 254 → 100 ms / 0 の順に試し、10 発/s で追従（失敗 0）を確認してこちらを既定にした（ユーザー決定）。acc 0 の行き過ぎ 22 カウント（ホイール 5.8°、許容 ±5.9°）で穴に止まるかはメカ組立後に確認し、だめなら 200 ms / 254（行き過ぎ 3）に戻す。逆転も 3000（1500 だと周期に追いつかず溜まる） |
| カウント端数 | `Feeder` が目標カウントを実数で累積し、`round(目標) − 送信済み` を送る（171, 170, 171, … が混ざる） |
| 起動 | 1 Hz で Ping → MODE（33）が 3 であることを確認（EEPROM は試験モードで設定済み。**ファームは書かない**。違えば警告して待つ）→ トルク ON → READY。応答が無い間は待つだけ（モータと同じく起動を止めない） |
| 状態 | READY の間 `FEED_STATUS_PERIOD_MS`（100 ms）ごとに `ReadStatus`（残りステップ・負荷・電流・温度）。Move の失敗が `FEED_FAIL_LIMIT`（5 回）続いたら NOT_READY に戻して再接続 |
| サーボ ID | 1（出荷時、`board::SERVO_ID`） |

やらないこと（別設計）: ジャム検知と自動復旧（設計メモ 5.5。ストッパー製作後のトルク試験で閾値を決めてから）、起動時の位相合わせ（メカ組立後）、PC への装填状態の返送、偽ファームの装填。

## 構造

```
Comm（MODE load）─▶ Controller::Tick（100 Hz）── load_mode()（途絶で 0）──▶ FeederTask（100 Hz）
                                                                              ├─ rabcl::Feeder::Update(mode, now) → steps
                                                                              └─ rabcl::St3215::Move(steps, speed, acc) / ReadStatus  ──▶ ServoBus（半二重 UART1）
```

### rabcl `controller/feeder.hpp`

```cpp
struct FeederConfig
{
  float counts_per_cell;  // サーボのカウント / 1 セル（gamma: 4096 / 8 / 3 = 170.67）
  float period_s;         // 1 セルの最短間隔 [s]
};

class Feeder
{
public:
  enum class Mode : uint8_t { STOP = 0, FORWARD = 1, REVERSE = 2 };
  explicit Feeder(const FeederConfig & config);
  // 毎 tick 呼ぶ。今サーボへ送る相対ステップ数（0 = 送らない）。
  // FORWARD / REVERSE への切り替わり（エッジ）で 1 セルを保留し、前のステップから period_s 経っていれば出す。
  // 押し続けている間は period_s ごとに 1 セル。保留はモードが STOP に戻っても残る
  int16_t Update(Mode mode, float now_s);
  void Cancel();                       // 保留を捨てる（途絶）
  float target_counts() const;         // 累積の目標（実数）
  int32_t sent_counts() const;         // 送った合計
  int32_t cells() const;               // 送ったセル数（逆転は負）
};
```

- 1 セル = `counts_per_cell` を目標に足し（逆転は引き）、`steps = lround(target) − sent`。
- 判定: `(pending || held) && (never || now − last_step ≥ period)`。`held` = mode が STOP でない。保留の向きはエッジ時のモード。

### gamma_turret_fw

- `board.hpp`: `SERVO_ID = 1`。
- `robot.hpp`: `FEED_CELLS = 8`、`FEED_GEAR_RATIO = 3.0f`、`FEED_COUNTS_PER_CELL = 4096 / 8 / 3`、`FEED_PERIOD_MS = 200`、`FEED_SPEED = 3000`、`FEED_ACC = 254`、`FEED_REVERSE_SPEED = 1500`、`FEED_RESPONSE_TIMEOUT_MS = 10`、`FEED_STATUS_PERIOD_MS = 100`、`FEED_FAIL_LIMIT = 5`。
- `Controller`: `CommandSnapshot::load`（0 / 1 / 2。途絶で 0）、`uint8_t load_mode() const`。
- `FeederTask`（`feeder_task.hpp` / `.cpp`）: `ServoBus` + `rabcl::St3215` + `rabcl::Feeder`。100 Hz の FreeRTOS タスク（優先度は制御タスクと同じ 5、スタック 4096）。状態 `NOT_READY / READY`、`Counters {state, cells, moves, move_fail, timeouts, last_error, status(St3215Status), status_ok}`。
- `main.cpp`: 生成・Start、1 Hz の `LogFeeder`。
- README: 装填行を 🔧、挙動・定数表・試験手順。

## 試験（サーボ単体、メカなし）

1. 24V（サーボの 7.4 V 系）を入れる前に P4 起動 → ログ `feeder: NOT_READY` が 1 Hz で続く。
2. サーボに電源 → `READY`（ping / mode 3 / torque on）。ホーンは止まったまま。
3. L1 を短く叩く → ホーンが約 15°（171 カウント）回る。ログ `cells=1`。
4. L1 を押し続ける → 200 ms ごとに回る（5 発/s）。`cells` が増える。
5. ○ → 逆方向に同じ周期で戻る。`cells` が減る。
6. 3 発で 171 + 170 + 171 = 512（`sent_counts`）。端数が溜まらない。
7. PC を止める → 止まる（途絶）。再開で続きから。
8. サーボの電源を切る → NOT_READY、入れ直す → READY に戻る。

## 実装者に任せること

- `FeederTask` のタスク本体の分割（Ping / 確認 / Move / ReadStatus）。
- ログの体裁。
