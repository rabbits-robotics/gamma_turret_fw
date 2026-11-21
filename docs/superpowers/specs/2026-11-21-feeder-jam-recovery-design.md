# 装填のジャム検知と自動復旧 設計書（gamma、2026-11-21）

球ありの装填試験（2026-11-21、組み替え後）で、約 420 セル送ったあと「上から落ちてくる球がホイールの縁に噛む」軽い詰まりが起きた。メカ側は直さず、ソフトで検知して自動で復旧する。

## 実測（2026-11-21、通常ビルド、球あり）

- 通常の送り（L1 押し続け 10 発/s）: 残りステップ約 147（1 セル = 170.67 未満）、負荷 −1〜−14 %、電流 0.44 A 以下。球なしのときと同じ軽さ。
- 詰まった瞬間（100 ms 周期の最初の読み）: 負荷 −100 %、電流 3.5 A、残りステップ 1713 → 1 秒後 2395。ファームは詰まっている間もステップを送り続けた（今回 4 セル、前回 22 セル）。
- 詰まって 2 秒でサーボの過負荷保護（err=0x20）が入り、トルクが 20 % に落ちて負荷 −20 % 固定になる。保護中はパッドの逆転も効かない。保護はトルク OFF → ON で解除される（Feetech の仕様。今回は未確認、試験 4 で確かめる）。
- ステップモード（MODE 3）で読む「現在位置」（レジスタ 56、`St3215Status::position`）は目標までの残りステップとして振る舞う（送ると増え、到達で 0、逆転中は負）。ファームのログはこれを `remaining` と呼んでいる。

## 現状のファーム

`FeederTask::Drive` は 10 ms ごとに `rabcl::Feeder::Update` から相対ステップを受け取ってそのまま `Move` し、100 ms ごとに `ReadStatus` で残り・負荷・電流・温度を読む。読んだ値はログに出すだけで判断に使っていない。

## 位相の考え方（鍵）

ステップモードの目標位置はサーボが積算している。起動時の位相合わせで目標は穴の上に来ており、その後はトルクを切らない限り「送った相対ステップの合計」だけ動く。**戻す量も進める量も 1 セルの整数倍にすれば目標は常に穴の上**に残り、絶対角を読み直す必要がない（ユーザーの指摘）。復旧中にトルクを切らない。

## 決めたこと

| 項目 | 決定 |
|---|---|
| 溜めない | \|残りステップ\| が `FEED_QUEUE_LIMIT_CELLS`（2 セル = 341 カウント）を超えている間は、残りと同じ向きの `Feeder` のステップを出させない（逆向きは残りを減らすので通す）。通常 10 発/s は 1 セル未満なので当たらない。しきい値判定が外れても（部分的な噛みで負荷が −60 % 程度など）キューが無限に伸びることを防ぐ。逆転で詰まったときも同じ（外れた瞬間に溜まった分だけ逆回転しない） |
| 検知 | 100 ms 周期の読みで \|負荷\| ≥ `FEED_JAM_LOAD`（800 = 80 %）または \|電流\| ≥ `FEED_JAM_CURRENT`（308 = 2.0 A）が `FEED_JAM_READS`（1）回、かつ残りステップ ≠ 0（動かそうとしている）。100 ms で検知し、保護の 2 秒に対して余裕がある。今日の詰まりは最初の読みで −100 %・3.5 A と明確だったので 1 回で判定する（ユーザー決定）。連続回数は設定として残し、誤検知が出たら 2 に上げる。向きは残りステップの符号で決める（正 = 正転の詰まり、負 = 逆転の詰まり）。操作者が L1 を離していてもサーボは残りを押し続けるので、送信の有無や load モードは見ない。逆転の詰まりも同じ条件で検知するが、復旧はしない（操作者がすでに介入している）：逆転ステップを止めるだけで、負荷が下がれば NORMAL に戻る |
| 戻す | 正転で検知したら `Move(−k × 171)` を 1 回。k = floor(残り / 170.67) + 1 で、目標が今の位置より後ろに来る最小のセル数（残り 147 + 2 セル溜まりなら k = 3）。相対ステップの合計は整数カウントで送るので、`Feeder` と同じ round 差分の作法で端数を溜めない（k セル分のカウント = lround(k × 170.67)）。戻しの目標が穴である保証は「送った累計が整数セル」で担保する |
| 進め直す | 残りが 0 になるか `FEED_RECOVERY_TIMEOUT_MS`（1000）経ったら `Move(+k セル)` で目標を元に戻す。残り 0 かつ負荷がしきい値未満で NORMAL に戻る。また詰まれば（同じ検知条件）試行回数 +1 で戻しからやり直す |
| あきらめ | 進め直しの失敗が `FEED_RETRY_LIMIT`（3）回になったら、もう一度 k セル戻して（残り 0 を待たず）JAMMED。遷移は 検知 → 戻し → 進め直し 1 失敗 → 戻し → 進め直し 2 失敗 → 戻し → 進め直し 3 失敗 → 戻し → JAMMED。`tries` は進め直しの失敗回数。JAMMED では正転ステップを出させず、○（逆転）のステップが来たら送って NORMAL に戻る（操作者が手で外して逆転で確認する流れ）。戻したままなので目標は穴の上、位相は保たれる |
| 保護が入ったとき | どの状態でも err に 0x20 が立っていたらトルクを切らないと動けないので、`FeederTask` を NOT_READY に落として既存の `Connect` を通す。**`Connect` の先頭に `SetTorque(false)` を足す**（今は起動時にトルクが切れている前提で呼んでいない。トルク ON のまま MODE 0 に切り替えると古い目標角へ飛ぶ恐れもある。通信途絶 5 回 → NOT_READY の既存経路でも同じく一度切ってから入れ直すことになるが、位相合わせが続くので問題ない。EEPROM 2 回）。ジャム起因の再接続では位相合わせを**後ろの穴**に限る（`Feeder::AlignStepsBackward`。次の穴が到達の許容（20）以内なら数カウントだけ進む。一番近い穴を選ぶと噛んだ球の方へ進み、再接続が 3 秒ごとに繰り返された: 実機 2026-11-21）。再接続も試行回数に数え、`FEED_RETRY_LIMIT` 回目で JAMMED（○ 待ち）。試行回数は詰まらずに 1 セル送れたら 0 に戻る。通信途絶からの再接続は数えない（`Reset`） |
| 到達の判定 | 静止時の残りは 0 にならず ±3 残る（実測）ので、\|残り\| ≤ `FEED_SETTLE_COUNTS`（5）で到達とみなす。詰まりの検知も残りがこの範囲なら数えない |
| タイムアウト | 進め直しの移動が `FEED_RECOVERY_TIMEOUT_MS` 以内に残り 0 にならなければ、その試行は失敗扱い（詰まりと同じ）。戻しがタイムアウトしたら（後ろにも動けない）目標が前にあるまま止まれないので、JAMMED を経由せずトルクを切って再接続（実機 2026-11-21: JAMMED で押し続けて保護に落ちた）。JAMMED に入ったあと最後の戻しが到達しないときも同じ。速度 3000 で 3 セル = 171 ms なので 1000 ms は十分 |
| Feeder の勘定 | 復旧の `Move` は `Feeder` を通さない（`sent_counts` と `cells` は操作者が要求したセルのまま）。戻し k と進め直し k が対になるので成功すれば差し引き 0。出させない向きは `Feeder::Update` に渡すモードを STOP に置き換える（ステップを作ってから捨てるのではなく作らせない。`Feeder` の実数累積が進まないので round の端数（最大 ±0.67 カウント/回）も溜まらず、`cells` も要求どおりの数のまま）。解除されたときの押し直しは `Feeder` のエッジ扱いで 1 セル保留になる |
| 置き場所 | 判定と状態機械は rabcl に `JamRecovery`（`include/rabcl/controller/jam_recovery.hpp`）として置き、gtest で検証する。入力は読んだ状態（残り・負荷・電流・err）と時刻、出力は「今やること」（何もしない / 正転ステップを捨てる / 相対 Move を送る / 再接続）。サーボや FreeRTOS には触れない。`FeederTask` が実行し、しきい値と回数は `robot.hpp` に置く（rabcl に機体の値を入れない） |
| 復旧の Move が通らないとき | UART で通らなかった戻し / 進め直しは次の tick で送り直す（送れないまま到達と見なすと、その分の球が出ない）。操作者の逆転・再接続で捨てる |
| 再接続に落とすとき | 検知した tick でトルクを切り、次の tick で `Connect`（1 s 待つと押し続けて保護に入る） |
| ログ | `main.cpp` の 1 Hz に `jam=<state> tries=<n> rec=<回数> jams=<回数> dropped=<n>` を足す。状態遷移は `FeederTask` から `ESP_LOGW`（JAM detected remaining=… load=… cur=… → BACK k=… → RETRY → OK / JAMMED） |

復旧中（BACKING / RETRYING）に ○（逆転）が来たら操作者優先: 逆転ステップをそのまま送り、復旧を中断して NORMAL に戻す（`tries` 0）。送った累計は整数セルのままなので位相は保たれる。復旧が 1 試行あたり最長 2 秒 × 3 回続く間、操作者の介入が効かないのを避ける。

やらないこと: PC への通知（PC → 機体の状態フレームにジャムのビットを足すのは別件）、逆転の自動復旧、パッドでの手動の位相微調整、しきい値の自動調整。

## rabcl `JamRecovery` のインターフェース（builder が決めてよい範囲で調整可）

```cpp
struct JamRecoveryConfig
{
  float counts_per_cell;     // 170.67
  int16_t load_limit;        // 0.1 %。|load| がこれ以上で詰まりの候補
  int16_t current_limit;     // 6.5 mA 単位
  uint8_t stall_reads;       // 連続でこの回数
  uint8_t queue_limit_cells; // 残りがこれを超えたら正転ステップを捨てる
  uint8_t retry_limit;       // 戻し→進め直しをこの回数まで
  float timeout_s;           // 戻し・進め直しの移動の待ち時間
};

class JamRecovery
{
public:
  enum class State : uint8_t { NORMAL, BACKING, RETRYING, JAMMED, RECONNECT };
  struct Action
  {
    bool block_forward;  // 正転のステップを出させない（Feeder に渡すモードを STOP にする）
    bool block_reverse;  // 逆転のステップを出させない
    int16_t move;        // 0 以外ならこの相対ステップを Move する（復旧の戻し / 進め直し）
    bool reconnect;      // FeederTask を NOT_READY に落とす
  };

  explicit JamRecovery(const JamRecoveryConfig & config);
  // 100 ms ごとの読みを入れる。error は ReadStatus 直後の St3215::last_error()（St3215Status に err は無い）
  Action Observe(int16_t remaining, int16_t load, int16_t current, uint8_t error, float now_s);
  // 逆転のステップを送ったら呼ぶ（JAMMED の解除、復旧中の中断）
  void NotifyReverse();
  // 再接続（Connect が通った）あとに呼ぶ。NORMAL、tries 0
  void Reset();
  State state() const;
  uint8_t tries() const;
  uint32_t recoveries() const;  // NORMAL に戻れた回数
  uint32_t jams() const;        // JAMMED になった回数
};
```

`block_forward` / `block_reverse` は `FeederTask` が毎 tick 参照する（`Observe` の直後だけでなく、次の `Observe` まで保持）。block_forward は NORMAL 以外の全状態と、NORMAL で残り > 上限のとき。block_reverse は NORMAL で残り < −上限のときだけ。

## 試験

rabcl（gtest、`test/test_jam_recovery.cpp`）:
1. 負荷 −100 % の読み 1 回で BACKING になり `move` = −lround(k × 170.67)、k = floor(remaining / 170.67) + 1（remaining 147 → k 1、490 → k 3）。stall_reads を 2 にした設定では 1 回では検知せず 2 回連続で検知する
2. 電流だけがしきい値超えでも検知する。残り 0 なら検知しない
3. BACKING で残り 0 → RETRYING で `move` = +同じカウント。残り 0 かつ負荷が下がれば NORMAL、`recoveries` +1、`tries` 0
4. RETRYING で再び検知 → tries +1 で BACKING。3 回目の失敗では `move` = −k セルを出してから JAMMED、`jams` +1。JAMMED では `block_forward` true、`NotifyReverse` で NORMAL
5. 残りが queue_limit を超えたら NORMAL でも `block_forward` true、下回れば false。残りが −queue_limit を下回れば `block_reverse` true
6. err に 0x20 → `reconnect` true、状態 RECONNECT。`Reset`（再接続後）で NORMAL、tries 0
7. RETRYING が timeout_s 以内に到達しなければ失敗として次の試行。BACKING がタイムアウトしたら RECONNECT。JAMMED で timeout_s 以内に到達しなければ RECONNECT
8. 残りが負（逆転中）の検知は `block_reverse` のみで `move` は 0、負荷が下がれば NORMAL
9. BACKING / RETRYING で `NotifyReverse` → NORMAL、tries 0、`move` 0
10. 送信の有無に関係なく残り > 0 と高負荷だけで検知する（L1 を離したあとの詰まり）

実機（組立済み機構）:
1. 球なしで L1 を押し続けながらホイールを指で軽く止める → 100 ms 程度で戻って再試行、指を離せば NORMAL（ログの rec +1）。止めたあとも穴に止まる（位相）
2. 止め続ける → 3 回で JAMMED、L1 が効かない、○ で逆転して NORMAL
3. 球ありで 10 発/s を続け、自然に噛んだときに自動復旧するか。復旧できず JAMMED になる率
4. 指で 3 秒以上止めて保護（err=0x20）を入れる → 再接続して READY に戻り、穴に合うか（トルク OFF で保護が消えることの確認）

## 検討した案

| 案 | 内容 | 判断 |
|---|---|---|
| A 検知して止めるだけ | 新しいステップを捨てて保護を避け、○ で人が外す | 試合中に毎回操作者の手が要る。却下 |
| B1 再接続で合わせ直す | トルクを切って `Connect`（MODE 0 で絶対角 → 一番近い穴） | 詰まるたびに EEPROM 2 回、前方に動くこともある。保護が入った場合のフォールバックにだけ使う |
| B2 残りから位置を逆算して戻す | 送った累計 − 残り = 今の位置、直前の穴までの戻し量を計算 | 計算は不要と分かった（下） |
| **B2' 整数セルで戻す（採用）** | トルクを切らなければ目標は穴の上。k セル戻して k セル進める | EEPROM なし、トルク断なし、常に後退。ユーザーの指摘で単純化 |
