// H1 的独立 C++ 通信进程：承担 DDS 状态接收和命令发送，Python runner 负责策略、状态机与运行保护。
// 接收 rt/lowstate 后，将最新状态、逐消息按键记录和通信统计写入共享内存，供 runner 读取。
// 通过本地 Unix 数据报接收 runner 的目标角度与 PD 增益，以固定频率及 NOW 请求发送 rt/lowcmd。
// 桥接看门狗监测 runner 命令超时、父进程断开和退出信号，触发后锁定为速度阻尼。
// 支持只读监听、CPU 绑核和实时调度；正常 QUIT 立即退出，父进程断开则先持续阻尼再退出。
// h1_bridge: C++ DDS I/O for scripts/deploy/bridge/h1_runner_bridge.py.
//
// This process does ALL rt/lowstate / rt/lowcmd traffic; the runner keeps obs, policy, safety and the state machine.
//
//   rt/lowstate (listener, every sample): CRC check (--crc), seqlock snapshot + per-sample ring in shm (h1_bridge_shm.h)
//   rt/lowcmd   (main thread)          : fixed --hz stream of the latest runner command + an immediate publish for
//                                        commands flagged NOW; nothing is written before the first runner command
//   rt/lowcmd   (watch reader)         : counts every sample (foreign-publisher watch, as the runner's watcher child did)
//   watchdog                           : no runner command for > --wd-ms, runner gone (stdin EOF) or SIGTERM once
//                                        publishing -> LATCHED damping (kp 0, kd --damp-kd, q measured) on the motors the
//                                        runner had enabled; runner gone/SIGTERM: damping for --orphan-s, then exit
//   QUIT command                       : stop publishing at once and exit (the runner already sent its final damping second)
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <string>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "h1_bridge_shm.h"

using namespace unitree::robot;
using unitree_go::msg::dds_::LowCmd_;
using unitree_go::msg::dds_::LowState_;

static double mono() {
  // 所有周期、心跳和超时使用单调时钟，与 Linux 上 Python time.monotonic() 的时间基准一致。
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Copied from unitree_sdk2 (https://github.com/unitreerobotics/unitree_sdk2),
// CRC implementation retained from the upstream Unitree SDK example code.
// Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"), BSD-3-Clause;
// see LICENSES/unitree_sdk2-BSD-3-Clause.txt and LICENSES/THIRD_PARTY_NOTICES.md.
static uint32_t Crc32Core(const uint32_t* ptr, uint32_t len) {
  // 按 32 位字计算 Unitree 消息 CRC；调用方传入的长度排除消息末尾的 CRC 字段本身。
  uint32_t xbit = 0, data = 0, CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1u << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

struct Args {
  // iface/domain 用于 DDS，shm/sock 用于本机进程通信；listen_only 禁止创建 DDS 命令写入通道。
  std::string iface, shm, sock;
  int domain = -1;
  bool crc = false, listen_only = false;
  // hz 为发送频率，wd_ms 为命令超时阈值，orphan_s 为父进程断开/退出信号后的阻尼持续时间。
  double hz = 500.0, wd_ms = 100.0, damp_kd = 3.0, orphan_s = 10.0;
  int cpu = -1, prio = 0;
};

// 状态区的浮点统计量逐字段原子写入；它们不与完整状态快照共用序列锁。
static void store_d(double* p, double v) { __atomic_store(p, &v, __ATOMIC_RELAXED); }

static H1BShm* S = nullptr;
static Args A;
static volatile sig_atomic_t g_sigterm = 0;

// 信号回调只置标志，实际切换阻尼和资源清理由主循环完成。
static void on_term(int) { g_sigterm = 1; }

// ------------------------------------------------------------------------------------------- rt/lowstate listener
static uint64_t g_n_rx = 0, g_n_bad = 0;
static uint32_t g_last_tick = 0;
static bool g_have_tick = false, g_tick_moving = false;
static double g_t_tick_change = 0.0;

static void snap_begin() {
  // 序列锁：写入开始时变奇数，完成后变偶数；读端要求复制前后序号相同且为偶数。
  __atomic_store_n(&S->snap_seq, S->snap_seq + 1, __ATOMIC_RELAXED);  // odd: writing (single writer)
  __atomic_thread_fence(__ATOMIC_RELEASE);
}
static void snap_end() { __atomic_store_n(&S->snap_seq, S->snap_seq + 1, __ATOMIC_RELEASE); }

static void on_lowstate(const void* p) {
  // DDS 接收回调只负责校验、复制和计数，不运行策略，也不在这里发布控制命令。
  const LowState_& m = *static_cast<const LowState_*>(p);
  const double t = mono();
  // 启用 --crc 时丢弃坏包，仅更新错误计数，不把坏包当作新的有效状态。
  if (A.crc && m.crc() != Crc32Core(reinterpret_cast<const uint32_t*>(&m), (sizeof(LowState_) >> 2) - 1)) {
    g_n_bad++;
    snap_begin();
    S->snap.n_crc_bad = g_n_bad;
    snap_end();
    return;
  }
  // 分别记录消息接收时间和机器人 tick 最近变化时间，供 runner 识别状态是否持续更新。
  const uint32_t tick = m.tick();
  if (!g_have_tick || tick != g_last_tick) {
    if (g_have_tick) g_tick_moving = true;
    g_have_tick = true;
    g_last_tick = tick;
    g_t_tick_change = t;
  }
  g_n_rx++;
  const auto& wr = m.wireless_remote();
  snap_begin();
  H1BSnap& s = S->snap;
  s.t_rx = t;
  s.t_tick_change = g_t_tick_change;
  s.n_rx = g_n_rx;
  s.n_crc_bad = g_n_bad;
  s.tick = tick;
  s.tick_moving = g_tick_moving ? 1u : 0u;
  s.reserved_mode = 0;
  s.reserved_control = 0;
  const auto& ms = m.motor_state();
  // 保留 GO 的 20 个硬件电机槽位；19 维策略关节重排由 Python 公共工具处理。
  for (int i = 0; i < H1B_NM; i++) {
    s.q[i] = ms[i].q();
    s.dq[i] = ms[i].dq();
    s.tau[i] = ms[i].tau_est();
    // 单温度值复制到两列以兼容状态结构；motorstate 存放 GO 丢包计数，而非故障位掩码。
    s.temp[i][0] = ms[i].temperature();
    s.temp[i][1] = ms[i].temperature();
    s.motorstate[i] = ms[i].lost();
  }
  // 原样保留躯干 IMU 和手柄数据；躯干到骨盆的 IMU 换算留给 Python 侧。
  const auto& imu = m.imu_state();
  for (int k = 0; k < 4; k++) s.quat[k] = imu.quaternion()[k];
  for (int k = 0; k < 3; k++) {
    s.gyro[k] = imu.gyroscope()[k];
    s.acc[k] = imu.accelerometer()[k];
    s.rpy[k] = imu.rpy()[k];
  }
  memcpy(s.wireless_remote, wr.data(), 40);
  snap_end();
  // 最新快照只保留一帧；环形缓冲另存每条被接受消息的时间、tick 和按键，便于逐样本去抖。
  // ring: invalidate, fill, publish index, advance head
  const uint64_t idx = g_n_rx - 1;
  // H1B_RING 为 2 的幂，可用按位与循环定位；先标记槽位无效，再填数据并发布索引。
  H1BRingEntry* e = &S->ring[idx & (H1B_RING - 1)];
  __atomic_store_n(&e->idx, ~0ull, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  e->t = t;
  e->tick = tick;
  e->reserved_mode = 0;
  e->keys = uint32_t(wr[2]) | (uint32_t(wr[3]) << 8);
  __atomic_store_n(&e->idx, idx, __ATOMIC_RELEASE);
  __atomic_store_n(&S->ring_head, idx + 1, __ATOMIC_RELEASE);
}

static void on_lowcmd_watch(const void*) {
  // 统计收到的所有 lowcmd（可能包含自身回送）；是否存在其他发布者由 runner 结合发送量判断。
  H1BStatus& st = S->status;
  store_d(&st.lc_t, mono());
  __atomic_fetch_add(&st.lc_count, 1, __ATOMIC_RELEASE);
}

// ------------------------------------------------------------------------------------------- setup helpers
static int usage() {
  fprintf(stderr,
          "usage: h1_bridge --domain D --shm NAME --sock PATH [--iface IF] [--crc] [--listen-only] [--hz 500] [--wd-ms 100] [--damp-kd 3]\n"
          "                 [--orphan-s 10] [--cpu N] [--prio P]\n");
  return 2;
}

static bool parse(int argc, char** argv) {
  // --crc/--listen-only 是无值开关，其余选项读取下一个参数；缺值或未知选项返回失败。
  for (int i = 1; i < argc; i++) {
    std::string k = argv[i];
    auto val = [&](void) -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    const char* v = nullptr;
    if (k == "--crc") { A.crc = true; continue; }
    if (k == "--listen-only") { A.listen_only = true; continue; }
    if (!(v = val())) return false;
    if (k == "--iface") A.iface = v;
    else if (k == "--domain") A.domain = atoi(v);
    else if (k == "--shm") A.shm = v;
    else if (k == "--sock") A.sock = v;
    else if (k == "--hz") A.hz = atof(v);
    else if (k == "--wd-ms") A.wd_ms = atof(v);
    else if (k == "--damp-kd") A.damp_kd = atof(v);
    else if (k == "--orphan-s") A.orphan_s = atof(v);
    else if (k == "--cpu") A.cpu = atoi(v);
    else if (k == "--prio") A.prio = atoi(v);
    else return false;
  }
  return A.domain >= 0 && !A.shm.empty() && !A.sock.empty() && A.hz > 0 && A.wd_ms > 0;
}

static H1BShm* map_shm(const std::string& name) {
  // 重新创建指定名称的 POSIX 共享内存，映射为 H1BShm：最新快照、按键环形缓冲和状态统计。
  shm_unlink(name.c_str());
  int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
  if (fd < 0) { perror("shm_open"); return nullptr; }
  if (ftruncate(fd, sizeof(H1BShm)) != 0) { perror("ftruncate"); return nullptr; }
  void* p = mmap(nullptr, sizeof(H1BShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (p == MAP_FAILED) { perror("mmap"); return nullptr; }
  H1BShm* s = static_cast<H1BShm*>(p);
  memset(s, 0, sizeof(H1BShm));
  for (int i = 0; i < H1B_RING; i++) s->ring[i].idx = ~0ull;
  s->status.bridge_pid = uint32_t(getpid());
  s->status.wd_s = A.wd_ms * 1e-3;
  s->version = H1B_VERSION;
  // 初始化完成后最后发布 magic，客户端据此识别已建立的协议内存布局。
  __atomic_store_n(&s->magic, H1B_MAGIC, __ATOMIC_RELEASE);
  return s;
}

static int open_sock(const std::string& path) {
  // 本地 Unix 数据报套接字保留命令包边界，非阻塞接收；这不是网络 UDP 端口。
  int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) { perror("socket"); return -1; }
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  if (path.size() >= sizeof(a.sun_path)) { fprintf(stderr, "socket path too long\n"); return -1; }
  strcpy(a.sun_path, path.c_str());
  unlink(path.c_str());
  if (bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) { perror("bind"); return -1; }
  return fd;
}

static void rt_setup() {
  // 为当前发布线程尝试绑核和 SCHED_FIFO 调度；失败只记录日志，不阻止程序继续运行。
  if (A.cpu >= 0) {
    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(A.cpu, &cs);
    int r = pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    fprintf(stderr, "[bridge] publisher thread affinity cpu %d: %s\n", A.cpu, r ? strerror(r) : "ok");
  }
  if (A.prio > 0) {
    sched_param sp{};
    sp.sched_priority = A.prio;
    int r = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    fprintf(stderr, "[bridge] publisher thread SCHED_FIFO %d: %s\n", A.prio, r ? strerror(r) : "ok");
  }
}

// ------------------------------------------------------------------------------------------- main
int main(int argc, char** argv) {
  if (!parse(argc, argv)) return usage();
  // Ctrl+C 交给 runner 组织正常阻尼退出；SIGTERM/SIGHUP 则由本进程的看门狗收尾。
  signal(SIGPIPE, SIG_IGN);
  signal(SIGINT, SIG_IGN);  // the runner owns Ctrl-C; it tells us to QUIT
  struct sigaction sa{};
  sa.sa_handler = on_term;
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGHUP, &sa, nullptr);

  // 先建立进程间通信，再启动 DDS 监听，使回调写入时共享内存已经就绪。
  S = map_shm(A.shm);
  if (!S) return 1;
  const int sfd = open_sock(A.sock);
  if (sfd < 0) return 1;

  if (A.iface.empty()) ChannelFactory::Instance()->Init(A.domain);
  else ChannelFactory::Instance()->Init(A.domain, A.iface);
  ChannelSubscriber<LowCmd_> watch("rt/lowcmd");
  watch.InitChannel(on_lowcmd_watch, 0);
  ChannelSubscriber<LowState_> sub("rt/lowstate");
  // 回调直接在 DDS listener 中执行，避免 SDK 另设队列积压状态样本。
  sub.InitChannel(on_lowstate, 0);  // 0: handler runs in the DDS listener (no SDK queue hiding stale samples)
  ChannelPublisher<LowCmd_> pub("rt/lowcmd");
  // 提前完成发布器初始化，但收到首条普通控制命令前不发送；只读模式连写入通道也不初始化。
  // writer exists from the start (creating it blocks 50-100 ms); it writes nothing before the first command.
  // --listen-only: no rt/lowcmd writer at all, commands are ignored (read-only checks on the real robot)
  if (!A.listen_only) pub.InitChannel();
  rt_setup();
  H1BStatus& st = S->status;
  // ready 只表示桥接器初始化完成；pub_started 才表示控制命令流已开始。
  __atomic_store_n(&st.ready, 1u, __ATOMIC_RELEASE);
  fprintf(stderr, "[bridge] up%s: domain %d iface '%s' crc %d hz %.0f wd %.0f ms damp kd %.1f shm %s sock %s pid %d\n", A.listen_only ? " (LISTEN ONLY: no rt/lowcmd writer)" : "", A.domain,
          A.iface.c_str(), int(A.crc), A.hz, A.wd_ms, A.damp_kd, A.shm.c_str(), A.sock.c_str(), int(getpid()));

  H1BCmd cmd{};       // latest runner command
  H1BCmd rx{};
  bool started = false, have_cmd = false, parent_gone = false, quit = false;
  double t_orphan = 0.0;
  const double period = 1.0 / A.hz, wd = A.wd_ms * 1e-3;
  double deadline = mono() + period;
  LowCmd_ out{};
  H1BSnap snap{};
  double t_last_pub = 0.0;

  auto publish = [&](void) {
    // 发送优先服从锁定的看门狗阻尼；否则使用最新 runner 命令及其 Q_MEASURED 标志。
    const bool damping = __atomic_load_n(&st.wd_fired, __ATOMIC_RELAXED) != 0;
    const bool need_q = damping || (cmd.flags & H1B_CMD_Q_MEASURED);
    // 需要实测角度时读取一致的快照，并确认至少接收过一条有效状态。
    bool have_q = need_q && h1b_snap_read(S, &snap, 100) == 0 && snap.n_rx > 0;
    out.head()[0] = 0xFE;
    out.head()[1] = 0xEF;
    out.level_flag() = 0xFF;
    out.gpio() = 0;
    auto& mc = out.motor_cmd();
    for (int i = 0; i < H1B_NM; i++) {
      const bool on = cmd.mode[i] != 0;
      mc[i].mode() = cmd.mode[i];
      // 目标速度与前馈力矩固定为零；正常控制依靠 q/kp/kd 形成位置 PD。
      mc[i].dq() = 0.0f;
      mc[i].tau() = 0.0f;
      if (damping) {
        // 看门狗阻尼：kp=0，仅给已启用电机设置 damp_kd，未启用槽位 kd=0。
        mc[i].q() = on && have_q ? float(snap.q[i]) : 0.0f;
        mc[i].kp() = 0.0f;
        mc[i].kd() = on ? float(A.damp_kd) : 0.0f;
      } else {
        // 正常命令可指定目标角，也可请求实测角；请求实测但读不到时将 kp 清零，避免拉向零位。
        mc[i].q() = need_q ? (on && have_q ? float(snap.q[i]) : 0.0f) : float(cmd.q[i]);
        mc[i].kp() = need_q && !have_q ? 0.0f : float(cmd.kp[i]);  // no measurement yet: never pull towards q = 0
        mc[i].kd() = float(cmd.kd[i]);
      }
    }
    // 每次根据实际发送内容重新计算 CRC；发送异常记为失败，供 runner 查看统计并处理。
    out.crc() = Crc32Core(reinterpret_cast<const uint32_t*>(&out), (sizeof(LowCmd_) >> 2) - 1);
    bool ok = false;
    const double tw = mono();
    try {
      ok = pub.Write(out);
    } catch (...) {
      ok = false;
    }
    const double tw1 = mono();
    // 记录最慢 Write、最大发送间隔、超周期次数及连续失败次数，用于诊断通信抖动。
    if (tw1 - tw > st.pub_write_max) store_d(&st.pub_write_max, tw1 - tw);
    if (t_last_pub > 0.0) {
      const double gap = tw - t_last_pub;
      if (gap > st.pub_gap_max) store_d(&st.pub_gap_max, gap);
      if (gap > 2.0 * period) __atomic_fetch_add(&st.n_gap_late, 1, __ATOMIC_RELAXED);
    }
    t_last_pub = tw;
    __atomic_fetch_add(&st.n_pub, 1, __ATOMIC_RELAXED);
    if (ok) {
      __atomic_store_n(&st.fail_consec, 0, __ATOMIC_RELAXED);
    } else {
      __atomic_fetch_add(&st.n_pub_fail, 1, __ATOMIC_RELAXED);
      __atomic_fetch_add(&st.fail_consec, 1, __ATOMIC_RELAXED);
    }
  };

  while (true) {
    // 每轮更新桥接心跳，并等待命令、父进程管道事件或下一个固定发送时刻。
    double now = mono();
    store_d(&st.t_hb, now);
    double rem = deadline - now;
    if (rem < 0) rem = 0;
    timespec to{time_t(rem), long((rem - double(time_t(rem))) * 1e9)};
    pollfd pf[2] = {{sfd, POLLIN, 0}, {0, POLLIN, 0}};
    int nfd = parent_gone ? 1 : 2;
    int pr = ppoll(pf, nfd, &to, nullptr);
    now = mono();
    bool now_flag = false;
    if (pr > 0 && (pf[0].revents & POLLIN)) {
      // 连续读完当前可用数据报，保留最后一条命令；其中任一 NOW 标志都请求本轮立即发布。
      while (true) {
        ssize_t n = recv(sfd, &rx, sizeof rx, 0);
        if (n < 0) break;
        if (n != ssize_t(sizeof rx) || rx.magic != H1B_MAGIC) {
          fprintf(stderr, "[bridge] dropped bad command datagram (%zd bytes)\n", n);
          continue;
        }
        // QUIT 在只读模式也有效；普通控制命令在只读模式直接忽略。
        if (rx.flags & H1B_CMD_QUIT) { quit = true; break; }
        if (A.listen_only) continue;
        cmd = rx;
        have_cmd = true;
        if (rx.flags & H1B_CMD_NOW) now_flag = true;
        __atomic_fetch_add(&st.n_cmd_rx, 1, __ATOMIC_RELAXED);
        store_d(&st.t_last_cmd, now);
      }
    }
    // stdin 是 runner 保持打开的管道；EOF 表示父进程已关闭连接，桥接器进入独立收尾流程。
    if (nfd == 2 && pr > 0 && (pf[1].revents & (POLLIN | POLLHUP | POLLERR))) {
      char b[64];
      ssize_t n = read(0, b, sizeof b);
      if (n <= 0) {
        parent_gone = true;
        fprintf(stderr, "[bridge] runner gone (stdin EOF)\n");
      }
    }
    if (quit) {
      // 正常退出协议要求 runner 已完成阻尼阶段；此处立即停止发布，不再额外等待 orphan_s。
      __atomic_store_n(&st.quit, 1u, __ATOMIC_RELEASE);
      fprintf(stderr, "[bridge] QUIT: %lu lowcmd published, %lu failed\n", (unsigned long)st.n_pub, (unsigned long)st.n_pub_fail);
      break;
    }
    if (have_cmd && !started) {
      // 首条命令才真正启动输出，并把截止时间设为当前时刻以立即发布。
      started = true;
      __atomic_store_n(&st.pub_started, 1u, __ATOMIC_RELEASE);
      fprintf(stderr, "[bridge] first runner command: rt/lowcmd stream starts\n");
      deadline = now;  // publish right away
    }
    if (!started) {
      // 从未发送过命令时，父进程断开或收到退出信号即可退出，无需主动发送阻尼。
      if (parent_gone || g_sigterm) break;  // nothing was ever published
      if (now >= deadline) deadline += period;
      continue;
    }
    // 这里监测 runner 命令新鲜程度，不是 lowstate 年龄；后者由 runner 的保护逻辑检查。
    // wd_fired 一旦置位就不复位，即使后续又收到正常命令，也继续输出看门狗阻尼。
    // watchdog (latched)
    if (!st.wd_fired) {
      const char* why = nullptr;
      if (now - st.t_last_cmd > wd) why = "runner silent";
      else if (parent_gone) why = "runner gone";
      else if (g_sigterm) why = "SIGTERM";
      if (why) {
        __atomic_store_n(&st.wd_fired, 1u, __ATOMIC_RELEASE);
        fprintf(stderr, "[bridge] WATCHDOG (%s, last command %.1f ms ago): latched damping kd %.1f\n", why,
                (now - st.t_last_cmd) * 1e3, A.damp_kd);
      }
    }
    // 只有父进程断开/退出信号才启动自动退出计时；单纯命令超时会持续保持阻尼。
    if ((parent_gone || g_sigterm) && t_orphan == 0.0) t_orphan = now;
    if (t_orphan > 0.0 && now - t_orphan > A.orphan_s) {
      fprintf(stderr, "[bridge] exit after %.1f s of damping without a runner\n", A.orphan_s);
      break;
    }
    // 固定周期与 NOW 请求共用同一发布函数；NOW 提前发送时不改变原有周期截止时间。
    if (now >= deadline || now_flag) publish();
    if (now >= deadline) {
      deadline += period;
      if (now - deadline > 10 * period) deadline = now + period;  // no burst after a stall
    }
  }
  // 移除本进程创建的通信入口，刷新日志后直接退出；跳过可能阻塞的 SDK 静态析构。
  close(sfd);
  unlink(A.sock.c_str());
  shm_unlink(A.shm.c_str());
  fflush(stderr);
  _exit(0);  // skip DDS teardown (the SDK's static destructors may block)
}
