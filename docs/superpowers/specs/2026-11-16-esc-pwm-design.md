# 射出 ESC の PWM 設計（gamma）

2026-11-16。射出ローラー × 2 を回す ESC（Motive-RC XR-7A、BLHeli_S、7 A、入力 1〜2S）に 50 Hz のサーボパルスを出す。rabcl_esp32 に `PwmOut`（LEDC）を足し、gamma_turret_fw の `Controller` が `fire` モードからパルス幅を決める。

## 目的と背景

- beta は STM32 の TIM1 で 50 Hz、起動時に 2000 → 1000 µs（各 3 s）の校正をしてから、`fire = 1` で 1200 µs、それ以外は 1000 µs を出していた。
- gamma は P4 が PC の USB で生きたまま 24V（ESC の電源系）だけ入ったり、P4 だけリセットされたりする。起動時に 2000 µs を 3 s 出すと、アーミング済みの ESC には**フルスロットル 3 秒**になる。
- BLHeli_S はスロットル範囲（既定 1040〜1960 µs）を EEPROM に保存するので、校正は毎回いらない。アーミングは min 未満のパルス（1000 µs）を受ければ済む。

## 決めたこと

| 項目 | 決定 |
|---|---|
| 起動時 | 起動直後から `FIRE_IDLE_US`（1000 µs）を出す。校正シーケンスは**やらない**。校正のコードも入れない（必要になったら BLHeli_S の手順を専用ビルドで足す） |
| 射出 | `fire = 1`（PC の MODE、R1 押下）で `FIRE_PULSE_US`（1200 µs、beta 実測値。gamma で再調整）。`fire = 0`、MODE 未受信、指令途絶（`WATCHDOG_S`）のどれでも 1000 µs |
| 上限 | パルス幅は `FIRE_PULSE_MAX_US`（1800 µs）以下に `static_assert`。校正に入る高いパルス（> 1900 µs 程度）を通常ファームが出せないようにする |
| 速度段 | 無し。`SPEED` モードは車体の歩き / 走りなのでローラーには使わない（昨日の TODO の「速度の段は MODE speed」は誤り） |
| 周期 | 50 Hz（beta と同じ。BLHeli_S の通常 PWM 1〜2 ms） |
| 2 本の扱い | 右（G0）と左（G1）に同じパルス幅。LEDC タイマー 1 本（`LEDC_TIMER_0`）をチャネル 0 / 1 で共有 |
| 分解能 | 14 bit（50 Hz で 1 LSB ≈ 1.22 µs。1000 → duty 819、1200 → 983） |
| ピン | 基板の回路図: Stamp-P4 **G0 = FIRE_RIGHT_MOTOR、G1 = FIRE_LEFT_MOTOR**（基板設計書 2026-08-10 の L6 / L5 と一致）。GPIO 直結、ESC 中央ピン NC |
| 置き場 | LEDC を包む `PwmOut` は rabcl_esp32（設計書 2026-10-20 に予定済み）。パルス幅を決めるのは `Controller::Tick`、ハードに書くのは `main.cpp` の制御タスク（Controller はハードを持たない） |

## 構造

```
Comm（MODE fire）─┐
                  ▼
Controller::Tick（100 Hz）── fire_pulse_us() ──▶ ControlTask（main.cpp）── PwmOut::SetPulseUs ──▶ G0 / G1 → ESC × 2
   途絶なら 1000
```

### rabcl_esp32 `PwmOut`

```cpp
struct PwmOutConfig
{
  gpio_num_t pin = GPIO_NUM_NC;
  ledc_timer_t timer = LEDC_TIMER_0;
  ledc_channel_t channel = LEDC_CHANNEL_0;
  uint32_t freq_hz = 50;
  ledc_timer_bit_t resolution = LEDC_TIMER_14_BIT;
  uint32_t initial_us = 0;  // Start() 直後に出すパルス幅（0 = Low のまま）
};

class PwmOut
{
public:
  explicit PwmOut(const PwmOutConfig & config);
  esp_err_t Start();                 // タイマー（LEDC_LOW_SPEED_MODE、LEDC_AUTO_CLK）とチャネルを設定し、initial_us を出す。2 回目は ESP_ERR_INVALID_STATE
  esp_err_t SetPulseUs(uint32_t us); // 未 Start は ESP_ERR_INVALID_STATE、周期を超える幅は ESP_ERR_INVALID_ARG。duty を書いて update
  uint32_t pulse_us() const;         // 最後に設定した幅
  uint32_t DutyFor(uint32_t us) const;  // us → duty（四捨五入）。テストと呼び出し側の確認用
};
```

- 同じタイマーを 2 つの `PwmOut` が設定してよい（同じ周波数 / 分解能なら ESP-IDF の `ledc_timer_config` は再設定を受け付ける）。
- 共通の決まりどおり、コンストラクタはハードを触らない。ホスト側ユニットテストは fake の `driver/ledc.h` で `ledc_timer_config` / `ledc_channel_config` / `ledc_set_duty` / `ledc_update_duty` の呼び出しを記録する。

### gamma_turret_fw

- `board.hpp`: `ESC_RIGHT_PIN = GPIO_NUM_0`、`ESC_LEFT_PIN = GPIO_NUM_1`、`ESC_PWM_HZ = 50`、`ESC_LEDC_TIMER = LEDC_TIMER_0`、`ESC_RIGHT_CHANNEL = LEDC_CHANNEL_0`、`ESC_LEFT_CHANNEL = LEDC_CHANNEL_1`。
- `robot.hpp`: `FIRE_IDLE_US = 1000`、`FIRE_PULSE_US = 1200`、`FIRE_PULSE_MAX_US = 1800`、`static_assert(FIRE_PULSE_US <= FIRE_PULSE_MAX_US)`。
- `Controller`: `CommandSnapshot` に `bool fire`。`Tick` で `GetMode(FIRE)` が 1 かつ途絶していないときだけ `fire = true`、`fire_pulse_us_` を 1200 / 1000 に置く。`uint32_t fire_pulse_us() const`。
- `main.cpp`: `PwmOut` × 2 を **app_main の最初**（CAN や Comm より前）に `Start()`（復電後に素早くアーミング）。`ControlTask` が `Tick` のあと毎回 2 本に `SetPulseUs(fire_pulse_us())`。`LogPc` に `fire=<us>` を出す。
- README: ペリフェラル表の ESC 行を 🔧、挙動と定数表に追記、校正しない理由と BLHeli_S の既定範囲、実機で 1200 を再調整する旨。

## 試験（実機、ESC + ローラー単体）

1. 24V（ESC の電源系）を入れる前に P4 を起動 → G0 / G1 に 50 Hz・1000 µs（オシロか、ESC のアーミング音で確認）。
2. ESC に電源 → アーミング音。ローラーは回らない。
3. R1 押下で回る（1200 µs）、離すと止まる。PC を止めて 0.5 s で止まる（途絶）。
4. P4 を USB でリセット → 起動中もローラーが回らない（起動直後から 1000 µs。フルスロットルにならない）。
5. 2 本の回転方向（ローラーは左右で逆回転のはず。逆なら ESC のモータ線 2 本を入れ替える）。
6. 1200 µs の回り方を beta と比べ、必要なら `FIRE_PULSE_US` を調整（未校正の ESC なら 1200 ≈ 17 %、beta の校正済みなら 20 %）。

## やらないこと

- 校正シーケンス、DShot、ローラーの速度段、ESC のテレメトリ。
- 装填（ST3215）の制御本体への組み込みは別設計。

## 実装者に任せること

- fake `driver/ledc.h` に置く型は使うものだけ（既存の fake と同じ流儀）。
- `ControlTask` へ `PwmOut` を渡す形（構造体 1 つを `arg` で渡す）。
