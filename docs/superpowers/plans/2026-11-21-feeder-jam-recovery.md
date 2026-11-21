# 装填のジャム検知と自動復旧 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 装填サーボの詰まりを 100 ms の状態読みで検知し、整数セルで戻して進め直す自動復旧（3 回で JAMMED、保護が入っていたら再接続）を rabcl + gamma_turret_fw に入れる。

**Architecture:** 判定と状態機械は rabcl の新クラス `JamRecovery`（入力: 残り・負荷・電流・err・時刻、出力: 出させない向き / 送る相対 Move / 再接続）。`FeederTask` が 100 ms の `ReadStatus` 直後に `Observe` を呼んで `Action` を保持し、毎 tick `Feeder::Update` に渡すモードをマスクし、`move` があれば `St3215::Move` する。しきい値と回数は `robot.hpp`。

**Tech Stack:** C++17、gtest（rabcl、ws_dev の colcon）、ESP-IDF / FreeRTOS（pioarduino、PowerShell で `pio` 1 つずつ）。

**Spec:** `fw_ws/gamma_turret_fw/docs/superpowers/specs/2026-11-21-feeder-jam-recovery-design.md`

## Global Constraints

- rabcl に機体の値を入れない。gamma の値は `robot.hpp` の `constexpr`（UPPER_SNAKE_CASE）。
- include は外部 `<>`、自リポジトリ `""`、グループを空行で分ける。
- rabcl の変更はユニットテスト必須。テストは `test/test_jam_recovery.cpp`、`CMakeLists.txt` の `ament_add_gtest` に登録。
- コミットは **ステージして diff を見せ、ユーザーの OK のあと**。メッセージは日本語 1 行、Co-Authored-By 無し。ファームは実機確認のあとにコミット。
- `remaining` = ステップモードの `St3215Status::position`（目標までの残り。正転で正、逆転で負、到達で 0）。
- 1 セル = 170.67 カウント（`4096 / 24`）。k セル分のカウントは `lround(k × 170.67)`。
- rabcl のテストは ws_dev で: `docker exec ws_dev bash -lc "source /opt/ros/jazzy/setup.bash; cd ~/rabbits_ws && colcon build --packages-select rabcl 2>&1 | tail -3 && ./build/rabcl/test_jam_recovery"`（ビルドは PowerShell から `docker exec` で。Git Bash は `/opt` を壊す）。

## Review Focus

1. 残りが上限ちょうど（341）や負荷がしきい値ちょうど（−800）のとき: 「超えたら」「以上」の境界で動作が変わる。Task 1 の境界テストで固定。
2. 検知の 100 ms 読みで `remaining` が 0 を飛び越して負になる（逆転の進め直しは無いが、戻し中に操作者が ○ を押した直後の読み）: `NotifyReverse` で NORMAL に戻っているので queue ルールだけが効く。Task 3 のテスト 9 のあとに残り負の読みを 1 回足す。
3. `Observe` の間（100 ms）に `Feeder` が 1 セル出してしまう: 検知と同じ読みで `block_forward` が立つので、次の tick から止まる。ファーム側の順序（Observe → Action 保持 → 次 tick でマスク）を Task 5 の実装手順で固定。
4. 戻しの `Move` が通らない（UART 失敗）: `move_fail` に数え、状態は BACKING のまま残り 0 待ち → タイムアウトで JAMMED。Task 5 の手順に明記。
5. 再接続後に `JamRecovery` を `Reset` しないと RECONNECT に留まり続ける: Task 5 で `Connect` 成功時に `Reset`。実機試験 4 で確認。

---

### Task 1: `JamRecovery` の検知とキュー上限（rabcl）

**Files:**
- Create: `rabbits_ws/src/rabcl/include/rabcl/controller/jam_recovery.hpp`
- Create: `rabbits_ws/src/rabcl/src/controller/jam_recovery.cpp`
- Modify: `rabbits_ws/src/rabcl/CMakeLists.txt`（15 行目付近と 53 行目付近の `src/controller/feeder.cpp` の隣に `jam_recovery.cpp`、132 行目付近に `ament_add_gtest(test_jam_recovery test/test_jam_recovery.cpp)` + `target_link_libraries`）
- Test: `rabbits_ws/src/rabcl/test/test_jam_recovery.cpp`

**Interfaces:**
- Produces:

```cpp
namespace rabcl {
struct JamRecoveryConfig {
  float counts_per_cell; int16_t load_limit; int16_t current_limit; uint8_t stall_reads;
  uint8_t queue_limit_cells; uint8_t retry_limit; float timeout_s;
};
class JamRecovery {
public:
  enum class State : uint8_t { NORMAL, BACKING, RETRYING, JAMMED, RECONNECT };
  struct Action { bool block_forward; bool block_reverse; int16_t move; bool reconnect; };
  explicit JamRecovery(const JamRecoveryConfig & config);
  Action Observe(int16_t remaining, int16_t load, int16_t current, uint8_t error, float now_s);
  void NotifyReverse();
  void Reset();
  State state() const; uint8_t tries() const; uint32_t recoveries() const; uint32_t jams() const;
};
}
```

- テストの共通設定（ファイル先頭の無名 namespace）: `JamRecoveryConfig{4096.0f / 24.0f, 800, 308, 1, 2, 3, 1.0f}` を返す `MakeConfig()`、`stall_reads` を 2 にした `MakeConfigTwoReads()`。

- [ ] **Step 1: 失敗するテストを書く**（ヘッダの構造体定義と空実装が無いので全部コンパイルエラーで落ちる）

```cpp
TEST(JamRecoveryDetect, QuietReadKeepsNormal) {
  JamRecovery r(MakeConfig());
  auto a = r.Observe(147, -140, 44, 0, 1.0f);   // 10 発/s の普段の読み
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
  EXPECT_FALSE(a.block_forward); EXPECT_FALSE(a.block_reverse); EXPECT_EQ(0, a.move); EXPECT_FALSE(a.reconnect);
}
TEST(JamRecoveryDetect, OneHighLoadReadBacksUp) {
  JamRecovery r(MakeConfig());
  auto a = r.Observe(147, -1000, 545, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::BACKING, r.state());
  EXPECT_EQ(-171, a.move);               // k = floor(147 / 170.67) + 1 = 1
  EXPECT_TRUE(a.block_forward);
}
TEST(JamRecoveryDetect, BackCountIsWholeCellsPastRemaining) {
  JamRecovery r(MakeConfig());
  EXPECT_EQ(-512, r.Observe(490, -1000, 0, 0, 1.0f).move);   // k = 3、lround(3 × 170.67) = 512
}
TEST(JamRecoveryDetect, CurrentAloneDetects) {
  JamRecovery r(MakeConfig());
  EXPECT_EQ(JamRecovery::State::BACKING, (r.Observe(147, -100, 308, 0, 1.0f), r.state()));
}
TEST(JamRecoveryDetect, LoadAtLimitDetectsBelowDoesNot) {
  JamRecovery a(MakeConfig()); a.Observe(147, -800, 0, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::BACKING, a.state());
  JamRecovery b(MakeConfig()); b.Observe(147, -799, 307, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::NORMAL, b.state());
}
TEST(JamRecoveryDetect, ZeroRemainingNeverDetects) {
  JamRecovery r(MakeConfig());
  r.Observe(0, -1000, 600, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
}
TEST(JamRecoveryDetect, TwoReadsConfigNeedsConsecutiveReads) {
  JamRecovery r(MakeConfigTwoReads());
  r.Observe(147, -1000, 0, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
  r.Observe(147, -100, 0, 0, 1.1f);     // 途切れたら数え直し
  r.Observe(147, -1000, 0, 0, 1.2f);
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
  r.Observe(147, -1000, 0, 0, 1.3f);
  EXPECT_EQ(JamRecovery::State::BACKING, r.state());
}
TEST(JamRecoveryDetect, ReverseStallBlocksReverseOnly) {
  JamRecovery r(MakeConfig());
  auto a = r.Observe(-150, 1000, 500, 0, 1.0f);
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
  EXPECT_TRUE(a.block_reverse); EXPECT_FALSE(a.block_forward); EXPECT_EQ(0, a.move);
  a = r.Observe(-150, 60, 10, 0, 1.1f);  // 負荷が下がれば解除
  EXPECT_FALSE(a.block_reverse);
}
TEST(JamRecoveryQueue, ForwardBlockedAboveLimit) {
  JamRecovery r(MakeConfig());
  EXPECT_FALSE(r.Observe(341, -100, 0, 0, 1.0f).block_forward);   // 2 セル = lround(341.33) ちょうどは通す
  EXPECT_TRUE(r.Observe(342, -100, 0, 0, 1.1f).block_forward);
  EXPECT_FALSE(r.Observe(342, -100, 0, 0, 1.1f).block_reverse);
  EXPECT_FALSE(r.Observe(170, -100, 0, 0, 1.2f).block_forward);
}
TEST(JamRecoveryQueue, ReverseBlockedBelowNegativeLimit) {
  JamRecovery r(MakeConfig());
  auto a = r.Observe(-342, 100, 0, 0, 1.0f);
  EXPECT_TRUE(a.block_reverse); EXPECT_FALSE(a.block_forward);
}
```

- [ ] **Step 2: 落ちることを確認**

Run: ws_dev のテストコマンド（Global Constraints）
Expected: `jam_recovery.hpp: No such file` でビルド失敗

- [ ] **Step 3: ヘッダと実装を書く**

`jam_recovery.hpp` に Interfaces の宣言（ファイル先頭に 3 行の説明コメント: 何を入れて何が出るか、整数セルで戻すので位相が保たれること、サーボには触れないこと）。`jam_recovery.cpp` に `Observe` の NORMAL 分岐のみ: `stalled = remaining != 0 && (|load| ≥ load_limit || |current| ≥ current_limit)`、連続回数を数えて `stall_reads` に達したら、`remaining > 0` なら `k = floor(remaining / counts_per_cell) + 1`、`back_counts_ = lround(k × counts_per_cell)`、`move = −back_counts_`、BACKING、`started_s_ = now_s`。`remaining < 0` なら `block_reverse` のみ。キュー上限: `limit = lround(queue_limit_cells × counts_per_cell)`、`remaining > limit` → `block_forward`、`remaining < −limit` → `block_reverse`。BACKING 以降は Task 2 で足す（今は BACKING に入ったら `block_forward` true を返すだけ）。`NotifyReverse` / `Reset` は Task 3 まで空でよいが宣言は入れる。

- [ ] **Step 4: 通ることを確認**

Run: 同上
Expected: `[  PASSED  ] 10 tests.`

- [ ] **Step 5: ステージして diff を見せる**（コミットはユーザーの OK 後。Task 3 まで 1 コミットにまとめてよい）

```bash
git -C rabbits_ws/src/rabcl add include/rabcl/controller/jam_recovery.hpp src/controller/jam_recovery.cpp test/test_jam_recovery.cpp CMakeLists.txt
```

---

### Task 2: 戻し → 進め直し → NORMAL / 失敗の繰り返し（rabcl）

**Files:**
- Modify: `rabbits_ws/src/rabcl/src/controller/jam_recovery.cpp`
- Test: `rabbits_ws/src/rabcl/test/test_jam_recovery.cpp`

**Interfaces:**
- Consumes: Task 1 の `JamRecovery`。
- Produces: 状態遷移 BACKING → RETRYING → NORMAL / BACKING、`recoveries()`、`tries()`。

- [ ] **Step 1: 失敗するテストを書く**

```cpp
TEST(JamRecoveryCycle, BackThenRetryThenNormal) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);                       // → BACKING, move −171
  auto a = r.Observe(-120, 300, 50, 0, 1.1f);                // 戻り中（残りは負）
  EXPECT_EQ(JamRecovery::State::BACKING, r.state()); EXPECT_EQ(0, a.move); EXPECT_TRUE(a.block_forward);
  a = r.Observe(0, -20, 0, 0, 1.2f);                         // 戻り切った
  EXPECT_EQ(JamRecovery::State::RETRYING, r.state()); EXPECT_EQ(171, a.move); EXPECT_TRUE(a.block_forward);
  a = r.Observe(90, -200, 30, 0, 1.3f);                      // 進め直し中
  EXPECT_EQ(JamRecovery::State::RETRYING, r.state()); EXPECT_EQ(0, a.move);
  a = r.Observe(0, -20, 0, 0, 1.4f);
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state()); EXPECT_FALSE(a.block_forward);
  EXPECT_EQ(1u, r.recoveries()); EXPECT_EQ(0, r.tries());
}
TEST(JamRecoveryCycle, StallDuringRetryBacksAgainWithSameCounts) {
  JamRecovery r(MakeConfig());
  r.Observe(490, -1000, 545, 0, 1.0f);                       // k = 3, −512
  r.Observe(0, -20, 0, 0, 1.2f);                             // → RETRYING +512
  auto a = r.Observe(512, -1000, 545, 0, 1.3f);              // 進まない
  EXPECT_EQ(JamRecovery::State::BACKING, r.state());
  EXPECT_EQ(-512, a.move);                                   // 残り 512 から k を数え直すと 4 セルになる。同じ 512 を戻す
  EXPECT_EQ(1, r.tries());
}
TEST(JamRecoveryCycle, RetryTimeoutCountsAsFailure) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  r.Observe(0, -20, 0, 0, 1.2f);                             // → RETRYING at 1.2
  r.Observe(60, -300, 50, 0, 2.1f);                          // 0.9 s: まだ待つ
  EXPECT_EQ(JamRecovery::State::RETRYING, r.state());
  auto a = r.Observe(60, -300, 50, 0, 2.21f);                // 1.0 s 超え
  EXPECT_EQ(JamRecovery::State::BACKING, r.state()); EXPECT_EQ(-171, a.move); EXPECT_EQ(1, r.tries());
}
TEST(JamRecoveryCycle, BackingTimeoutGivesUp) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  auto a = r.Observe(-100, 900, 400, 0, 2.01f);
  EXPECT_EQ(JamRecovery::State::JAMMED, r.state()); EXPECT_EQ(0, a.move); EXPECT_EQ(1u, r.jams());
}
```

- [ ] **Step 2: 落ちることを確認**

Run: ws_dev のテストコマンド
Expected: 4 件 FAIL（状態が BACKING のまま / `move` が 0）

- [ ] **Step 3: `Observe` に BACKING / RETRYING の分岐を足す**

BACKING: `remaining == 0` → RETRYING、`move = +back_counts_`、`started_s_ = now_s`、連続回数を 0 に。`now_s − started_s_ > timeout_s` → JAMMED、`++jams_`。RETRYING: 検知（Task 1 と同じ条件と回数）または `now_s − started_s_ > timeout_s` → `++tries_`、`move = −back_counts_`、BACKING、`started_s_ = now_s`（`tries_ ≥ retry_limit` の分岐は Task 3）。`remaining == 0` かつ検知していない → NORMAL、`++recoveries_`、`tries_ = 0`。BACKING / RETRYING / JAMMED / RECONNECT では `block_forward = true`。

- [ ] **Step 4: 通ることを確認**

Run: 同上
Expected: `[  PASSED  ] 14 tests.`

- [ ] **Step 5: ステージ**（Task 1 と同じファイル）

---

### Task 3: あきらめ、操作者優先、保護からの再接続（rabcl）

**Files:**
- Modify: `rabbits_ws/src/rabcl/src/controller/jam_recovery.cpp`
- Modify: `rabbits_ws/src/rabcl/README.md:12`（`controller/` 行に `JamRecovery` を追記）
- Test: `rabbits_ws/src/rabcl/test/test_jam_recovery.cpp`

**Interfaces:**
- Consumes: Task 2 の遷移。
- Produces: JAMMED / RECONNECT、`NotifyReverse()`、`Reset()`、`jams()`。

- [ ] **Step 1: 失敗するテストを書く**

```cpp
namespace { void FailRetry(JamRecovery & r, float t) { r.Observe(0, -20, 0, 0, t); r.Observe(171, -1000, 545, 0, t + 0.1f); } }
TEST(JamRecoveryGiveUp, ThirdFailureBacksOffThenJams) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);                       // 検知、戻し
  FailRetry(r, 1.2f); EXPECT_EQ(1, r.tries()); EXPECT_EQ(JamRecovery::State::BACKING, r.state());
  FailRetry(r, 1.5f); EXPECT_EQ(2, r.tries());
  r.Observe(0, -20, 0, 0, 1.8f);                             // → RETRYING 3 回目
  auto a = r.Observe(171, -1000, 545, 0, 1.9f);              // 3 回目の失敗
  EXPECT_EQ(JamRecovery::State::JAMMED, r.state());
  EXPECT_EQ(-171, a.move);                                   // 最後も戻してから JAMMED
  EXPECT_TRUE(a.block_forward); EXPECT_EQ(1u, r.jams()); EXPECT_EQ(3, r.tries());
  a = r.Observe(0, -20, 0, 0, 2.5f);                         // JAMMED は残り 0 でも抜けない
  EXPECT_EQ(JamRecovery::State::JAMMED, r.state()); EXPECT_TRUE(a.block_forward); EXPECT_FALSE(a.block_reverse);
}
TEST(JamRecoveryGiveUp, ReverseClearsJammed) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  FailRetry(r, 1.2f); FailRetry(r, 1.5f); FailRetry(r, 1.8f);
  ASSERT_EQ(JamRecovery::State::JAMMED, r.state());
  r.NotifyReverse();
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state()); EXPECT_EQ(0, r.tries());
  EXPECT_FALSE(r.Observe(-171, 100, 20, 0, 2.0f).block_forward);
}
TEST(JamRecoveryOperator, ReverseDuringRecoveryAbortsToNormal) {
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);                       // BACKING
  r.NotifyReverse();
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state());
  auto a = r.Observe(-200, 100, 20, 0, 1.1f);                // 残りが負でも queue ルールだけ
  EXPECT_FALSE(a.block_forward); EXPECT_FALSE(a.block_reverse); EXPECT_EQ(0, a.move);
  r.Observe(147, -1000, 545, 0, 2.0f); r.Observe(0, -20, 0, 0, 2.2f);   // RETRYING
  r.NotifyReverse();
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state()); EXPECT_EQ(0, r.tries());
}
TEST(JamRecoveryProtect, OverloadFlagRequestsReconnectFromAnyState) {
  JamRecovery r(MakeConfig());
  auto a = r.Observe(2410, -200, 20, 0x20, 1.0f);            // NORMAL で保護
  EXPECT_TRUE(a.reconnect); EXPECT_TRUE(a.block_forward); EXPECT_EQ(0, a.move);
  EXPECT_EQ(JamRecovery::State::RECONNECT, r.state());
  a = r.Observe(0, -20, 0, 0, 1.1f);                         // Reset まで留まる
  EXPECT_EQ(JamRecovery::State::RECONNECT, r.state()); EXPECT_TRUE(a.block_forward);
  r.Reset();
  EXPECT_EQ(JamRecovery::State::NORMAL, r.state()); EXPECT_EQ(0, r.tries());
  JamRecovery s(MakeConfig());
  s.Observe(147, -1000, 545, 0, 1.0f);                       // BACKING で保護
  EXPECT_TRUE(s.Observe(-171, -200, 20, 0x20, 1.1f).reconnect);
  EXPECT_EQ(JamRecovery::State::RECONNECT, s.state());
}
TEST(JamRecoveryDetect, DetectsWithoutAnySend) {             // L1 を離したあとの詰まり: 入力は残りと負荷だけ
  JamRecovery r(MakeConfig());
  r.Observe(147, -100, 0, 0, 1.0f);
  r.Observe(147, -1000, 545, 0, 1.1f);
  EXPECT_EQ(JamRecovery::State::BACKING, r.state());
}
```

- [ ] **Step 2: 落ちることを確認**

Run: ws_dev のテストコマンド
Expected: 5 件 FAIL

- [ ] **Step 3: 実装**

`Observe` の先頭: `error & 0x20` → RECONNECT、`reconnect = true`、`block_forward = true`、`move = 0`（どの状態からでも。RECONNECT 中は毎回同じ Action を返し、`reconnect` は最初の 1 回だけ true でよい）。RETRYING の失敗分岐: `++tries_` のあと `tries_ ≥ retry_limit` なら `move = −back_counts_` で JAMMED、`++jams_`（戻しの完了は待たない）。JAMMED: `block_forward = true`、`move = 0`、`remaining` で抜けない。`NotifyReverse`: BACKING / RETRYING / JAMMED → NORMAL、`tries_ = 0`、連続回数 0（NORMAL / RECONNECT では何もしない）。`Reset`: NORMAL、`tries_ = 0`、連続回数 0。README の `controller/` 行末に「`JamRecovery`（装填のジャム検知と復旧の状態機械: 残り・負荷・電流から詰まりを検知し、整数セル戻して進め直す。3 回で JAMMED、過負荷保護なら再接続。サーボには触れない）」。

- [ ] **Step 4: 通ることを確認（全 suite）**

Run: ws_dev で `colcon build --packages-select rabcl && colcon test --packages-select rabcl && colcon test-result --verbose | tail -3`
Expected: `test_jam_recovery` 19 tests PASSED、全体 0 failures（既存 379 + 19）

- [ ] **Step 5: ステージして diff を見せ、OK 後にコミット・push**

```bash
git -C rabbits_ws/src/rabcl add -A include/rabcl/controller/jam_recovery.hpp src/controller/jam_recovery.cpp test/test_jam_recovery.cpp CMakeLists.txt README.md
# OK 後: git commit -m "JamRecovery を追加（装填のジャム検知と整数セルの復旧）" && git push
```

---

### Task 4: `FeederTask` への組み込み（gamma_turret_fw）

**Files:**
- Modify: `fw_ws/gamma_turret_fw/src/robot.hpp:197-200`（`FEED_FAIL_LIMIT` のあと）
- Modify: `fw_ws/gamma_turret_fw/src/feeder_task.hpp`（ヘッダコメント、`Counters`、メンバ）
- Modify: `fw_ws/gamma_turret_fw/src/feeder_task.cpp`（`Connect`、`Drive`、`counters`）
- Modify: `fw_ws/gamma_turret_fw/src/main.cpp:157-171`（`LogFeeder`）
- Modify: `fw_ws/gamma_turret_fw/components/rabcl`（サブモジュールを Task 3 のコミットへ）
- Modify: `fw_ws/gamma_turret_fw/README.md:53,144,160,186`（装填行、`feeder_task` 行、操作の装填項、定数表）
- Test: 実機（下の Step 6）。ファームにユニットテストは無い（rabcl 側で済ませる）

**Interfaces:**
- Consumes: `rabcl::JamRecovery`（Task 1〜3）、`rabcl::St3215::Move / SetTorque / ReadStatus / last_error`、`rabcl::Feeder::Update / Cancel`。
- Produces: `FeederTask::Counters` に `rabcl::JamRecovery::State jam_state; uint8_t jam_tries; uint32_t recoveries; uint32_t jams; uint32_t recovery_moves;`。

- [ ] **Step 1: `robot.hpp` に定数を足す**

```cpp
// ジャム検知と自動復旧（docs/superpowers/specs/2026-11-21-feeder-jam-recovery-design.md）。球ありの実測: 普段は負荷 −14 % 以下・0.44 A 以下、詰まると −100 %・3.5 A
constexpr int16_t FEED_JAM_LOAD = 800;             // [0.1 %] |負荷| がこれ以上で詰まり
constexpr int16_t FEED_JAM_CURRENT = 308;          // [6.5 mA] = 2.0 A
constexpr uint8_t FEED_JAM_READS = 1;              // 100 ms の読みで連続この回数（誤検知が出たら 2）
constexpr uint8_t FEED_QUEUE_LIMIT_CELLS = 2;      // 残りがこれを超えたら同じ向きのステップを出さない
constexpr uint8_t FEED_RETRY_LIMIT = 3;            // 進め直しの失敗がこの回数で JAMMED
constexpr uint32_t FEED_RECOVERY_TIMEOUT_MS = 1000; // [ms] 戻し・進め直しの移動の待ち
```

- [ ] **Step 2: `feeder_task.hpp`**

`#include <rabcl/controller/jam_recovery.hpp>`。メンバ `rabcl::JamRecovery recovery_;`、`rabcl::JamRecovery::Action action_ = {};`、`uint32_t recovery_moves_ = 0;`。`Counters` に Produces の 5 つ。ヘッダコメントに「100 ms の状態読みごとに `JamRecovery::Observe`、出させない向きは `Feeder` に渡すモードを STOP に置き換え、復旧の Move は `Feeder` を通さない」と設計書へのリンクを 1 行足す。コンストラクタで `recovery_(rabcl::JamRecoveryConfig{robot::FEED_COUNTS_PER_CELL, robot::FEED_JAM_LOAD, robot::FEED_JAM_CURRENT, robot::FEED_JAM_READS, robot::FEED_QUEUE_LIMIT_CELLS, robot::FEED_RETRY_LIMIT, robot::FEED_RECOVERY_TIMEOUT_MS / 1000.0f})`。

- [ ] **Step 3: `feeder_task.cpp`**

`Connect`: `Ping` の直後に `if (!servo_.SetTorque(false)) { return false; }`。成功の末尾（`align_*` を書くところ）で `recovery_.Reset(); action_ = {};`。
`Drive`: (1) `mode = ToFeederMode(...)`、`if (action_.block_forward && mode == FORWARD) mode = STOP;`、`if (action_.block_reverse && mode == REVERSE) mode = STOP;`。(2) `steps = feeder_.Update(mode, now_s)`、`steps < 0` なら送る前に `recovery_.NotifyReverse()` と `action_.block_forward = false`（次の Observe まで待たず、状態に合わせる）。(3) 送信は今のまま。(4) 100 ms の `ReadStatus` が通ったら `const auto before = recovery_.state(); action_ = recovery_.Observe(st.position, st.load, st.current, servo_.last_error(), now_s);`。`action_.move != 0` → `servo_.Move(action_.move, action_.move > 0 ? robot::FEED_SPEED : robot::FEED_REVERSE_SPEED, robot::FEED_ACC)`、通れば `++recovery_moves_`、失敗は `++move_fail_`（状態は変えない。残り 0 待ち → タイムアウトで JAMMED に落ちる）。`action_.reconnect` → 既存の NOT_READY に落とす処理と同じ（`state_ = NOT_READY`、`status_ok_ = false`、`feeder_.Cancel()`、`last_connect_ms_ = now_ms`）、`ESP_LOGW(TAG, "overload protection (err=0x%02X): reconnecting", ...)`。状態が変わったら `ESP_LOGW(TAG, "jam: %s -> %s remaining=%d load=%d cur=%d move=%+d tries=%u", ...)`（状態名は `const char *` の表を無名 namespace に）。
`counters`: 5 つをコピー（`recovery_` のゲッターはロック外でよい。`Feeder` と同じ扱い）。

- [ ] **Step 4: `main.cpp` の `LogFeeder`**

READY 行の `fail=%lu` の後ろに ` jam=%s tries=%u rec=%lu jams=%lu rmoves=%lu` を足す（状態名の表は `feeder_task.hpp` に `static const char * StateName(rabcl::JamRecovery::State)` を置いて共用）。

- [ ] **Step 5: サブモジュール更新とビルド**

```powershell
git -C fw_ws/gamma_turret_fw/components/rabcl fetch; git -C fw_ws/gamma_turret_fw/components/rabcl checkout <Task 3 のコミット>
cd fw_ws/gamma_turret_fw; & "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e gamma_turret 2>&1 | Select-Object -Last 5
```

Expected: `SUCCESS`。警告 0（`-Wall`）。

- [ ] **Step 6: 実機試験（ユーザーと。設計書の実機 1〜4）**

書き込み前にシリアル取り込みを止める（`Get-Process powershell` で `capture_com3.ps1` を kill）→ `pio run -e gamma_turret -t upload` → 取り込み再開 → launch。
1. 球なし、L1 押し続け + 指で軽く止める → ログ `jam: NORMAL -> BACKING ... -> RETRYING -> NORMAL`、`rec` +1、離したあと穴に止まる
2. 止め続ける → `tries` 1, 2, 3 → `JAMMED`、L1 無視、○ で `NORMAL`
3. 球あり 10 発/s、自然な噛みで `rec` が増え JAMMED にならないか（なった回数を記録）
4. 指で 3 秒以上止める → `overload protection ... reconnecting` → `READY ... align=` → 穴に合う
Expected: 4 つとも設計書どおり。外れたらしきい値（`FEED_JAM_*`）か回数を直して再試験。

- [ ] **Step 7: README とチェックリスト**

README: 53 行目の装填行に「ジャム検知（100 ms、負荷 80 % / 2 A）→ 整数セル戻して進め直し、3 回で JAMMED、保護なら再接続」と試験日、144 行目の `feeder_task` 行に `JamRecovery` と設計書リンク、160 行目「ジャム検知・自動復旧…未実装」を実装済みの説明に、定数表 186 行目の後ろに `FEED_JAM_LOAD / FEED_JAM_CURRENT / FEED_JAM_READS`、`FEED_QUEUE_LIMIT_CELLS`、`FEED_RETRY_LIMIT / FEED_RECOVERY_TIMEOUT_MS` の 3 行。`docs/2026-10-25-control-bringup-checklist.md` に追記 9（組み替え後の球あり結果と実機試験 1〜4 の結果）。

- [ ] **Step 8: ステージして diff を見せ、OK 後にコミット・push**

```bash
git -C fw_ws/gamma_turret_fw add src/robot.hpp src/feeder_task.hpp src/feeder_task.cpp src/main.cpp components/rabcl README.md docs/
# OK 後: git commit -m "装填のジャム検知と自動復旧を追加" && git push
```

---

## 自己レビュー

- 設計書の各項目 → 溜めない (T1)、検知 (T1)、戻す (T1)、進め直す (T2)、あきらめ (T3)、保護 (T3 + T4 Connect のトルク OFF)、タイムアウト (T2)、Feeder の勘定（T4 のモード置き換え）、置き場所 (T1〜3 / T4 robot.hpp)、ログ (T4)、復旧中の ○ (T3 + T4 の NotifyReverse)。実機試験 1〜4 は T4 Step 6。
- 型: `Action{block_forward, block_reverse, move, reconnect}`、`Observe(int16_t, int16_t, int16_t, uint8_t, float)` を全タスクで同じに使っている。`tries()` は `uint8_t`、テストは `EXPECT_EQ(1, r.tries())` で int と比較（gtest は昇格で通る）。
- 境界: 341 / 342、−800 / −799、308 / 307、タイムアウト 0.9 / 1.01 s をテストで固定。
