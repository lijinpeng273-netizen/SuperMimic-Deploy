// Read-only H1 GO odometry -> same-PC UDP. No DDS writers or robot service calls.
// Retain DDS SampleInfo (SDK message-only callbacks discard source time/identity).
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <arpa/inet.h>
#include <signal.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

using unitree_go::msg::dds_::LowState_;
using unitree_go::msg::dds_::SportModeState_;
static volatile sig_atomic_t stopped = 0;
static void stop(int) { stopped = 1; }
static double mono() {
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Unitree SDK BSD-3-Clause CRC algorithm; see LICENSES/unitree_sdk2-BSD-3-Clause.txt.
static uint32_t crc32(const uint32_t* words, size_t count) {
  uint32_t crc = 0xffffffff;
  for (size_t i = 0; i < count; ++i) {
    for (uint32_t bit = 1u << 31; bit; bit >>= 1) {
      bool high = crc & 0x80000000;
      crc <<= 1;
      if (high) crc ^= 0x04c11db7;
      if (words[i] & bit) crc ^= 0x04c11db7;
    }
  }
  return crc;
}

struct SourceGuard {
  dds::core::InstanceHandle publisher{dds::core::null};
  int64_t last_ns = 0;
  double last_received = 0;
  double best_offset = std::numeric_limits<double>::infinity();
  double lag = 0;
  bool accept(const dds::sub::SampleInfo& info, double received) {
    const auto ts = info.timestamp();
    const int64_t ns = int64_t(ts.sec()) * 1000000000LL + ts.nanosec();
    if (ns <= 0) throw std::runtime_error("DDS source timestamp unavailable");
    if (last_ns && info.publication_handle() != publisher)
      throw std::runtime_error("DDS publisher changed; restart calibration/controller");
    if (ns < last_ns) throw std::runtime_error("DDS source time regressed");
    if (ns == last_ns) return false;
    if (last_ns && (ns - last_ns) * 1e-9 > received - last_received + 0.1)
      throw std::runtime_error("DDS source clock jumped forward");
    publisher = info.publication_handle();
    last_ns = ns;
    last_received = received;
    const double offset = received - ns * 1e-9;
    best_offset = std::min(best_offset, offset);
    lag = offset - best_offset;
    if (lag > 0.1) throw std::runtime_error("DDS relative transport/source lag exceeded 100 ms");
    return true;
  }
};

struct Low { int64_t ns; double received, waist; };
struct Odom {
  int64_t ns = 0;
  double received = 0;
  std::array<float, 3> p{};
  std::array<float, 4> q{};
};
template<class A> static bool finite(const A& a) {
  return std::all_of(a.begin(), a.end(), [](double x) { return std::isfinite(x); });
}

int main(int argc, char** argv) {
  std::string iface, topic = "rt/odommodestate", udp = "127.0.0.1:15101";
  int domain = -1, fd = -1;
  double seconds = 0;
  bool check_only = false;
  try {
    for (int i = 1; i < argc; ++i) {
      std::string k = argv[i];
      if (k == "--help") {
        printf("Read-only, no DDS writers. Same-PC UDP, raw odom-frame position/orientation + paired waist.\n"
               "usage: h1_odom_bridge --iface IF --domain D [--udp 127.0.0.1:15101]\n"
               "       [--odom-topic rt/odommodestate] [--seconds 0] [--check-only]\n"
               "Pair <= 5 ms; CRC required; fresh source <= 100 ms; output <= 100 Hz.\n");
        return 0;
      }
      if (k == "--check-only") { check_only = true; continue; }
      if (++i >= argc) throw std::runtime_error("missing argument value");
      std::string v = argv[i];
      if (k == "--iface") iface = v;
      else if (k == "--domain") domain = std::stoi(v);
      else if (k == "--udp") udp = v;
      else if (k == "--odom-topic") topic = v;
      else if (k == "--seconds") seconds = std::stod(v);
      else throw std::runtime_error("unknown option: " + k);
    }
    if (iface.empty() || domain < 0 || !std::isfinite(seconds) || seconds < 0)
      throw std::runtime_error("require --iface, nonnegative --domain and --seconds");
    if (udp.substr(0, 10) != "127.0.0.1:")
      throw std::runtime_error("UDP must be loopback: clocks must share this PC");
    const int port = std::stoi(udp.substr(10));
    if (port < 1 || port > 65535) throw std::runtime_error("invalid UDP port");
    sockaddr_in target{};
    target.sin_family = AF_INET; target.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &target.sin_addr);
    if (!check_only) {
      fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (fd < 0) throw std::runtime_error("cannot open UDP socket");
    }
    signal(SIGINT, stop); signal(SIGTERM, stop);
    auto factory = unitree::robot::ChannelFactory::Instance();
    factory->Init(domain, iface);
    // Empty callbacks leave native readers available for take() with SampleInfo.
    auto lc = factory->CreateRecvChannel<LowState_>("rt/lowstate", {});
    auto oc = factory->CreateRecvChannel<SportModeState_>(topic, {});
    auto lr = lc->GetReader()->GetNative();
    auto odr = oc->GetReader()->GetNative();
    SourceGuard low_guard, odom_guard;
    std::deque<Low> lows;
    Odom odom;
    uint32_t last_tick = 0;
    bool have_tick = false;
    uint64_t sequence = 0, bad_crc = 0;
    int64_t sent_ns = 0;
    const double begin = mono();
    const std::string session = std::to_string(getpid()) + "-" + std::to_string(int64_t(begin * 1e9));
    double next_send = begin, next_status = begin, last_pair = begin;
    printf("[odom] READ ONLY domain=%d iface=%s topic=%s; %s; no DDS command writer\n",
           domain, iface.c_str(), topic.c_str(), check_only ? "check only, no UDP output" : udp.c_str());
    fflush(stdout);
    while (!stopped && (seconds == 0 || mono() - begin < seconds)) {
      for (const auto& sample : lr.take()) {
        if (!sample.info().valid()) continue;
        const auto& m = sample.data();
        if (m.crc() != crc32(reinterpret_cast<const uint32_t*>(&m), (sizeof(LowState_) >> 2) - 1)) {
          ++bad_crc; continue;
        }
        double now = mono(), waist = m.motor_state()[6].q();
        if (!std::isfinite(waist)) continue;
        if (!low_guard.accept(sample.info(), now)) continue;
        const uint32_t tick = m.tick();
        if (have_tick) {
          const uint32_t delta = tick - last_tick;
          if (delta == 0) continue;
          if (delta >= 0x80000000u) throw std::runtime_error("lowstate tick regressed/reset");
        }
        have_tick = true; last_tick = tick;
        lows.push_back({low_guard.last_ns, now - low_guard.lag, waist});
        while (lows.size() > 128) lows.pop_front();
      }
      for (const auto& sample : odr.take()) {
        if (!sample.info().valid()) continue;
        const auto& m = sample.data();
        const auto& q = m.imu_state().quaternion();
        if (m.error_code() || !finite(m.position()) || !finite(q)) continue;
        double n = 0; for (double v : q) n += v * v;
        if (std::abs(std::sqrt(n) - 1) > 0.01) continue;
        const double now = mono();
        if (!odom_guard.accept(sample.info(), now)) continue;
        odom = {odom_guard.last_ns, now - odom_guard.lag, m.position(), q};
      }
      const double now = mono();
      if (now >= next_send && odom.ns > sent_ns && !lows.empty()) {
        auto best = std::min_element(lows.begin(), lows.end(), [&](const Low& a, const Low& b) {
          return std::abs(a.ns - odom.ns) < std::abs(b.ns - odom.ns);
        });
        const double pair_ms = std::abs(best->ns - odom.ns) * 1e-6;
        const double sample_time = std::min(best->received, odom.received);
        if (pair_ms <= 5.0 && now - sample_time <= 0.1) {
          ++sequence;
          char packet[1024];
          const int n = snprintf(packet, sizeof(packet),
              "{\"frame_id\":\"unitree_odom_v1\",\"session\":\"%s\",\"sequence\":%llu,"
              "\"source_ns\":%lld,\"sample_monotonic_s\":%.9f,\"pair_ms\":%.6f,"
              "\"position\":[%.9g,%.9g,%.9g],\"quaternion_wxyz\":[%.9g,%.9g,%.9g,%.9g],\"waist_q\":%.9g}",
              session.c_str(), (unsigned long long)sequence, (long long)odom.ns, sample_time, pair_ms,
              odom.p[0], odom.p[1], odom.p[2], odom.q[0], odom.q[1], odom.q[2], odom.q[3], best->waist);
          if (n < 0 || size_t(n) >= sizeof(packet)) throw std::runtime_error("packet overflow");
          if (!check_only && sendto(fd, packet, n, 0, reinterpret_cast<sockaddr*>(&target), sizeof(target)) != n)
            throw std::runtime_error("UDP send failed");
          sent_ns = odom.ns; next_send = now + 0.01; last_pair = now;
          if (now >= next_status) {
            printf("[odom] paired=%llu z=%.6f m waist=%.4f rad pair=%.3f ms age=%.2f ms CRC_bad=%llu\n",
                   (unsigned long long)sequence, odom.p[2], best->waist, pair_ms,
                   (now - sample_time) * 1000, (unsigned long long)bad_crc);
            fflush(stdout); next_status = now + 3;
          }
        }
      }
      // No continuing with a silently restarted/paused estimator and old transform.
      if (sequence && now - last_pair > 0.1)
        throw std::runtime_error("no fresh synchronized odometry for 100 ms");
      if (!sequence && now - begin > 5)
        throw std::runtime_error("no valid synchronized lowstate/odom within 5 s");
      usleep(1000);
    }
    if (fd >= 0) { close(fd); fd = -1; }
    if (!sequence) throw std::runtime_error("no valid odometry pair collected");
    printf("[odom] stopped, paired=%llu CRC_bad=%llu; no files written\n",
           (unsigned long long)sequence, (unsigned long long)bad_crc);
    return 0;
  } catch (const std::exception& e) {
    fprintf(stderr, "[odom] STOP: %s\n", e.what());
    if (fd >= 0) close(fd);
    return 1;
  }
}
