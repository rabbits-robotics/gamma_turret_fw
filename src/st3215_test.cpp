// ST3215 の試験モード（pio run -e st3215_test）。USB コンソールから 1 行コマンドでサーボを操作し、
// 装填機構の設計メモ 5.1 の 4 項目（多回転、到達判定、ステップ所要時間、負方向）を測る。
// 制御本体（CAN、IMU）は起動しない。設計書: rabcl docs/superpowers/specs/2026-10-25-st3215-design.md 4.3
#ifdef GAMMA_ST3215_TEST

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <driver/usb_serial_jtag.h>
#include <driver/usb_serial_jtag_vfs.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rabcl/component/st3215.hpp>

#include "board.hpp"
#include "servo_bus.hpp"

namespace
{

constexpr const char * TAG = "st3215";
constexpr uint8_t SERVO_ID = 1;
constexpr uint32_t RESPONSE_TIMEOUT_MS = 10;
constexpr uint32_t MOVE_TIMEOUT_MS = 2000;
constexpr uint32_t TRACE_MS = 10;
constexpr int DONE_TOLERANCE = 5;  // 残りステップがこれ以下で MOVING 0 なら到達とみなす（カウント）
constexpr uint16_t DEFAULT_SPEED = 1000;
constexpr uint8_t DEFAULT_ACC = 50;

using P = rabcl::St3215Protocol;

ServoBus bus;
rabcl::St3215 servo(bus, SERVO_ID, RESPONSE_TIMEOUT_MS);

uint32_t NowMs()
{
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// USB コンソールにキー入力が来ていれば 1 バイト返す（ブロックしない）。
// 改行は無視する: CRLF 端末では fgets が CR で返ったあと LF が 1 バイト残り、それで止まってしまう
bool PollKey(char & c)
{
  uint8_t b;
  while (usb_serial_jtag_read_bytes(&b, 1, 0) > 0) {
    if (b == '\r' || b == '\n') {
      continue;
    }
    c = static_cast<char>(b);
    return true;
  }
  return false;
}

void PrintHelp()
{
  printf(
    "commands:\n"
    "  i                      ping + dump registers 3..70\n"
    "  m <0-3>                mode (0 position, 1 wheel, 2 pwm, 3 step). lock 0 -> mode -> lock 1\n"
    "  t <0|1>                torque off/on\n"
    "  s <steps> [spd] [acc]  one move (step mode: relative). default spd %u acc %u\n"
    "  l <n> <steps> <ms>     repeat s every <ms> ms, n times (waits for each move)\n"
    "  b <n> <steps> <ms> [spd] [acc]  burst: send every <ms> ms without waiting\n"
    "  p [ms]                 stream status every ms (default 50). any key stops\n"
    "  w <reg> <val>          write 1 byte\n"
    "  r <reg> [len]          read len bytes (default 1)\n"
    "  h                      this help\n",
    DEFAULT_SPEED, DEFAULT_ACC);
}

void CmdInfo()
{
  if (!servo.Ping()) {
    printf("ping: NO RESPONSE (timeouts=%lu)\n", static_cast<unsigned long>(servo.timeouts()));
    return;
  }
  printf("ping: ok (error=0x%02X)\n", servo.last_error());
  // 3..70 を 16 バイトずつ
  uint8_t regs[71] = {};
  for (uint8_t reg = 3; reg <= 70;) {
    const uint8_t len = static_cast<uint8_t>((70 - reg + 1) < 16 ? (70 - reg + 1) : 16);
    if (!servo.Read(reg, regs + reg, len)) {
      printf("read %u..%u failed\n", reg, reg + len - 1);
      return;
    }
    reg = static_cast<uint8_t>(reg + len);
  }
  printf("model=%u id=%u baud=%u(0=1Mbps) angle_limit=%u..%u ofs=%d mode=%u torque=%u acc=%u lock=%u\n",
    P::GetU16(regs + P::REG_MODEL), regs[P::REG_ID], regs[P::REG_BAUD],
    P::GetU16(regs + P::REG_MIN_ANGLE), P::GetU16(regs + P::REG_MAX_ANGLE),
    P::DecodeSigned(P::GetU16(regs + P::REG_OFS)), regs[P::REG_MODE], regs[P::REG_TORQUE_ENABLE],
    regs[P::REG_ACC], regs[P::REG_LOCK]);
  printf("goal pos=%d time=%u speed=%u torque_limit=%u\n",
    P::DecodeSigned(P::GetU16(regs + P::REG_GOAL_POSITION)), P::GetU16(regs + P::REG_GOAL_TIME),
    P::GetU16(regs + P::REG_GOAL_SPEED), P::GetU16(regs + P::REG_TORQUE_LIMIT));
  printf("present pos=%d speed=%d load=%d volt=%.1fV temp=%uC moving=%u current=%d\n",
    P::DecodeSigned(P::GetU16(regs + P::REG_PRESENT_POSITION)),
    P::DecodeSigned(P::GetU16(regs + P::REG_PRESENT_SPEED)),
    P::DecodeLoad(P::GetU16(regs + P::REG_PRESENT_LOAD)), regs[P::REG_PRESENT_VOLTAGE] / 10.0,
    regs[P::REG_PRESENT_TEMPERATURE], regs[P::REG_MOVING],
    P::DecodeSigned(P::GetU16(regs + P::REG_PRESENT_CURRENT)));
}

// 1 回の移動。送信から Moving が 0 に戻るまで状態を読み続け、所要時間と行き過ぎを表す
// accepted にはサーボが受理したステップ量（残りレジスタの最大値）を返す
bool CmdStep(int steps, unsigned speed, unsigned acc, int * accepted)
{
  rabcl::St3215Status before;
  const uint32_t timeouts0 = servo.timeouts();
  if (!servo.ReadStatus(before)) {
    printf("read before move failed (timeouts=%lu)\n", static_cast<unsigned long>(servo.timeouts()));
    return false;
  }
  const uint32_t t0 = NowMs();
  if (!servo.Move(static_cast<int16_t>(steps), static_cast<uint16_t>(speed),
      static_cast<uint8_t>(acc)))
  {
    printf("move failed (error=0x%02X timeouts=%lu)\n", servo.last_error(),
      static_cast<unsigned long>(servo.timeouts()));
    return false;
  }
  uint32_t t_start = 0;  // Moving が 1 になった時刻
  uint32_t t_stop = 0;   // Moving が 0 に戻った時刻
  int extreme = before.position;
  rabcl::St3215Status st = before;
  int reads = 0;
  int fails = 0;
  uint32_t next_trace = 0;  // TRACE_MS ごとに位置と速度を 1 サンプル出す（位置レジスタの意味を見るため）
  printf("  trace(t:pos/speed/moving):");
  // 待ち上限 = 基本 2 s + 等速で進む時間
  const uint32_t move_timeout_ms =
    MOVE_TIMEOUT_MS + 1000u * static_cast<uint32_t>(steps < 0 ? -steps : steps) / (speed ? speed : 1);
  while (NowMs() - t0 < move_timeout_ms) {
    // 待ちは入れない（tick 10 ms なので 2 ms 待ちは作れない）。応答待ちで CPU は空く。実測 約 0.45 ms / 読み
    if (!servo.ReadStatus(st)) {
      ++fails;
      continue;
    }
    ++reads;
    if (NowMs() - t0 >= next_trace) {
      printf(" %lu:%d/%d/%u", static_cast<unsigned long>(NowMs() - t0), st.position, st.speed,
        st.moving ? 1 : 0);
      next_trace += TRACE_MS;
    }
    if (steps >= 0 ? st.position > extreme : st.position < extreme) {  // 残りの最大 = 受理されたステップ量
      extreme = st.position;
    }
    if (st.moving && t_start == 0) {
      t_start = NowMs();
    }
    // 実測: 送信から約 10 ms の間、残りがステップ量のまま MOVING が 0 に戻る瞬間がある。残りも見る
    const int remaining = st.position < 0 ? -st.position : st.position;
    if (!st.moving && t_start != 0 && remaining <= DONE_TOLERANCE) {
      t_stop = NowMs();
      break;
    }
  }
  // ステップモードの位置レジスタ（56）は「残りステップ数」（実測: 送信直後にステップ量、停止で 0 付近）
  printf("\nstep=%+d speed=%u acc=%u | moving %lums..%lums t_move=%lums | remaining %d->%d (max %d) | reads=%d fails=%d timeouts=%lu err=0x%02X%s\n",
    steps, speed, acc, static_cast<unsigned long>(t_start ? t_start - t0 : 0),
    static_cast<unsigned long>(t_stop ? t_stop - t0 : 0),
    static_cast<unsigned long>((t_start && t_stop) ? t_stop - t_start : 0), before.position,
    st.position, extreme, reads, fails,
    static_cast<unsigned long>(servo.timeouts() - timeouts0), servo.last_error(),
    t_stop ? "" : " TIMEOUT(moving never settled)");
  if (accepted) {
    *accepted = extreme;
  }
  return t_stop != 0;
}

void CmdLoop(int n, int steps, unsigned period_ms)
{
  long sum_accepted = 0;
  int ok = 0;
  for (int i = 0; i < n; ++i) {
    const uint32_t t0 = NowMs();
    int accepted = 0;
    if (CmdStep(steps, DEFAULT_SPEED, DEFAULT_ACC, &accepted)) {
      ++ok;
    }
    sum_accepted += accepted;
    char c;
    if (PollKey(c)) {
      printf("stopped by key\n");
      break;
    }
    const uint32_t spent = NowMs() - t0;
    if (spent < period_ms) {
      vTaskDelay(pdMS_TO_TICKS(period_ms - spent));
    }
  }
  const long expected = static_cast<long>(n) * steps;
  printf("total: n=%d ok=%d expected=%+ld accepted=%+ld diff=%+ld\n", n, ok, expected, sum_accepted,
    sum_accepted - expected);
}

// 連射: 到達を待たずに period_ms ごとに n 回ステップを送る。移動中に受けた指令が積算されるか（残りが増えるか）、
// 置き換えられるかを残りの軌跡で見る。最後に全部止まるまでの時間を出す
void CmdBurst(int n, int steps, unsigned period_ms, unsigned speed, unsigned acc)
{
  const uint32_t t0 = NowMs();
  const uint32_t end_ms = static_cast<uint32_t>(n) * period_ms + 3000;
  int sent = 0;
  int send_failed = 0;
  uint32_t next_send = 0;
  uint32_t next_trace = 0;
  int max_remaining = 0;
  uint32_t settled_at = 0;
  rabcl::St3215Status st{};
  printf("  trace(t:remaining/moving, S=send):");
  while (NowMs() - t0 < end_ms) {
    const uint32_t t = NowMs() - t0;
    if (sent < n && t >= next_send) {
      if (!servo.Move(static_cast<int16_t>(steps), static_cast<uint16_t>(speed),
          static_cast<uint8_t>(acc)))
      {
        ++send_failed;
      }
      ++sent;
      next_send += period_ms;
      printf(" S%d@%lu", sent, static_cast<unsigned long>(t));
    }
    if (!servo.ReadStatus(st)) {
      continue;
    }
    const int remaining = st.position < 0 ? -st.position : st.position;
    if (remaining > max_remaining) {
      max_remaining = remaining;
    }
    if (t >= next_trace) {
      printf(" %lu:%d/%u", static_cast<unsigned long>(t), st.position, st.moving ? 1 : 0);
      next_trace += TRACE_MS;
    }
    if (sent == n && t >= next_send - period_ms + 20 && !st.moving && remaining <= DONE_TOLERANCE) {
      settled_at = t;
      break;
    }
  }
  printf("\nburst: n=%d steps=%+d period=%ums speed=%u acc=%u | sent=%d failed=%d max_remaining=%d settled_at=%lums%s (if accumulated, total = %+ld)\n",
    n, steps, period_ms, speed, acc, sent, send_failed, max_remaining,
    static_cast<unsigned long>(settled_at), settled_at ? "" : " TIMEOUT",
    static_cast<long>(n) * steps);
}

void CmdStream(unsigned period_ms)
{
  printf("streaming every %u ms (press any key to stop)\n", period_ms);
  const uint32_t t0 = NowMs();
  while (true) {
    rabcl::St3215Status st;
    if (servo.ReadStatus(st)) {
      printf("t=%6lums pos=%6d speed=%6d load=%5d volt=%.1fV temp=%uC moving=%u current=%d\n",
        static_cast<unsigned long>(NowMs() - t0), st.position, st.speed, st.load,
        st.voltage / 10.0, st.temperature, st.moving ? 1 : 0, st.current);
    } else {
      printf("t=%6lums NO RESPONSE\n", static_cast<unsigned long>(NowMs() - t0));
    }
    char c;
    if (PollKey(c)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}

void Dispatch(char * line)
{
  char cmd = 0;
  long a = 0, b = 0, c = 0, d = 0, e = 0;
  const int n = sscanf(line, " %c %ld %ld %ld %ld %ld", &cmd, &a, &b, &c, &d, &e);
  if (n < 1) {
    return;
  }
  switch (cmd) {
    case 'i': CmdInfo(); break;
    case 'm':
      if (n < 2 || a < 0 || a > 3) {
        printf("usage: m <0-3>\n");
        break;
      }
      {
        const uint32_t timeouts0 = servo.timeouts();
        const bool ok = servo.SetMode(static_cast<P::Mode>(a));
        printf("set mode %ld: %s (timeouts %lu, error=0x%02X)\n", a, ok ? "ok" : "FAILED",
          static_cast<unsigned long>(servo.timeouts() - timeouts0), servo.last_error());
      }
      {
        uint8_t v = 0xFF;
        if (servo.ReadU8(P::REG_MODE, v)) {
          printf("mode now %u\n", v);
        }
      }
      break;
    case 't':
      printf("torque %ld: %s\n", a, servo.SetTorque(a != 0) ? "ok" : "FAILED");
      break;
    case 's':
      if (n < 2) {
        printf("usage: s <steps> [speed] [acc]\n");
        break;
      }
      CmdStep(static_cast<int>(a), n >= 3 ? static_cast<unsigned>(b) : DEFAULT_SPEED,
        n >= 4 ? static_cast<unsigned>(c) : DEFAULT_ACC, nullptr);
      break;
    case 'l':
      if (n < 4) {
        printf("usage: l <n> <steps> <period_ms>\n");
        break;
      }
      CmdLoop(static_cast<int>(a), static_cast<int>(b), static_cast<unsigned>(c));
      break;
    case 'b':
      if (n < 4) {
        printf("usage: b <n> <steps> <period_ms> [speed] [acc]\n");
        break;
      }
      CmdBurst(static_cast<int>(a), static_cast<int>(b), static_cast<unsigned>(c),
        n >= 5 ? static_cast<unsigned>(d) : DEFAULT_SPEED, n >= 6 ? static_cast<unsigned>(e) : DEFAULT_ACC);
      break;
    case 'p': CmdStream(n >= 2 ? static_cast<unsigned>(a) : 50); break;
    case 'w':
      if (n < 3) {
        printf("usage: w <reg> <val>\n");
        break;
      }
      printf("write reg %ld = %ld: %s (error=0x%02X)\n", a, b,
        servo.WriteU8(static_cast<uint8_t>(a), static_cast<uint8_t>(b)) ? "ok" : "FAILED",
        servo.last_error());
      break;
    case 'r': {
      const uint8_t len = static_cast<uint8_t>(n >= 3 ? b : 1);
      uint8_t buf[16] = {};
      if (n < 2 || len > 16 || !servo.Read(static_cast<uint8_t>(a), buf, len)) {
        printf("read failed\n");
        break;
      }
      printf("reg %ld:", a);
      for (uint8_t i = 0; i < len; ++i) {
        printf(" %02X", buf[i]);
      }
      if (len == 2) {
        printf("  (u16 %u, s16 %d)", P::GetU16(buf), P::DecodeSigned(P::GetU16(buf)));
      }
      printf("\n");
      break;
    }
    case 'h': PrintHelp(); break;
    default: printf("unknown command '%c'\n", cmd); PrintHelp(); break;
  }
}

}  // namespace

extern "C" void app_main(void)
{
  usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
  usb_serial_jtag_vfs_use_driver();
  setvbuf(stdin, nullptr, _IONBF, 0);

  ESP_LOGI(TAG, "ST3215 test mode: UART%d tx=G%d rx=G%d %lu bps, servo id %u",
    static_cast<int>(board::SERVO_UART_PORT), static_cast<int>(board::SERVO_UART_TX_PIN),
    static_cast<int>(board::SERVO_UART_RX_PIN), static_cast<unsigned long>(board::SERVO_UART_BAUD),
    SERVO_ID);
  ESP_ERROR_CHECK(bus.Start());
  PrintHelp();

  char line[64];
  while (true) {
    printf("> ");
    fflush(stdout);
    if (fgets(line, sizeof(line), stdin) == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    Dispatch(line);
  }
}

#endif  // GAMMA_ST3215_TEST
