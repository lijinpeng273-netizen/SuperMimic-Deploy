// Shared-memory / socket protocol between h1_bridge (C++, unitree_sdk2 DDS I/O) and the Python runner
// (scripts/deploy/h1_runner.py, via the ctypes client libh1bridge_client.so built from h1_bridge_client.cpp).
//
//   bridge -> runner : POSIX shm /dev/shm/<name>, layout H1BShm below.
//                      snap   : latest CRC-good rt/lowstate (seqlock, written by the DDS listener thread)
//                      ring   : per-sample (t, tick, reserved_mode, remote keys) of every accepted lowstate, so the runner
//                               runs its button debouncer / reserved_mode check on every sample as before
//                      status : counters of the lowcmd publisher / watchdog / rt/lowcmd watch (single atomics)
//   runner -> bridge : unix datagram socket <sock>, one H1BCmd per datagram (latest wins).
//
// All times are CLOCK_MONOTONIC seconds (= Python time.monotonic() on Linux).
#pragma once
#include <stdint.h>

#define H1B_MAGIC 0x48314231u  // "H1B1"
#define H1B_VERSION 2u
#define H1B_NM 20               // motors in unitree_go LowState_/LowCmd_
#define H1B_RING 256            // power of two

// ---------------------------------------------------------------------------------------- bridge -> runner
typedef struct {
  double t_rx;           // receive time of this sample
  double t_tick_change;  // last time lowstate.tick changed
  uint64_t n_rx;         // accepted samples so far
  uint64_t n_crc_bad;    // dropped (bad CRC) samples so far
  uint32_t tick;
  uint32_t tick_moving;  // 1 once tick has changed at least once
  uint32_t reserved_mode;
  uint32_t reserved_control;
  double q[H1B_NM];
  double dq[H1B_NM];
  double tau[H1B_NM];
  double temp[H1B_NM][2];
  uint32_t motorstate[H1B_NM];
  uint32_t _pad0;
  double quat[4];  // w x y z
  double gyro[3];
  double acc[3];
  double rpy[3];
  uint8_t wireless_remote[40];
} H1BSnap;

typedef struct {
  uint64_t idx;  // sample index (= n_rx - 1 at write time)
  double t;
  uint32_t tick;
  uint32_t reserved_mode;
  uint32_t keys;  // wireless_remote[2] | wireless_remote[3] << 8
  uint32_t _pad;
} H1BRingEntry;

typedef struct {
  double t_hb;            // publisher thread heartbeat (every loop)
  uint64_t n_pub;         // rt/lowcmd Write() calls
  uint64_t n_pub_fail;    // Write() returned false
  uint64_t fail_consec;   // consecutive failed writes
  uint64_t n_cmd_rx;      // H1BCmd datagrams received from the runner
  double t_last_cmd;      // receive time of the last H1BCmd
  uint64_t lc_count;      // rt/lowcmd samples seen by the watch reader (all publishers, incl. our own if delivered)
  double lc_t;            // time of lc_count
  uint32_t ready;         // 1 once readers + socket are up
  uint32_t pub_started;   // 1 once the rt/lowcmd stream runs (first H1BCmd)
  uint32_t wd_fired;      // latched: runner silent > watchdog (or gone) -> bridge publishes damping itself
  uint32_t quit;          // 1 after a QUIT command (no more rt/lowcmd from the bridge)
  uint32_t bridge_pid;
  uint32_t _pad;
  double wd_s;            // watchdog limit
  double pub_gap_max;     // largest interval between consecutive rt/lowcmd writes since the stream started [s]
  uint64_t n_gap_late;    // intervals > 2 periods
  double pub_write_max;   // slowest Write() call [s]
} H1BStatus;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint64_t snap_seq;   // seqlock: odd while the listener writes snap
  H1BSnap snap;
  uint64_t ring_head;  // next sample index to be written
  H1BRingEntry ring[H1B_RING];
  H1BStatus status;
} H1BShm;

// ---------------------------------------------------------------------------------------- runner -> bridge
#define H1B_CMD_NOW 1u          // publish right away (policy step, DAMP) in addition to the fixed-rate stream
#define H1B_CMD_Q_MEASURED 2u   // q = latest measured lowstate q (PASSIVE / DAMP)
#define H1B_CMD_QUIT 4u         // stop publishing and exit

typedef struct {
  uint32_t magic;
  uint32_t flags;
  uint64_t seq;
  uint32_t reserved_mode;
  uint32_t _pad;
  uint8_t mode[H1B_NM];
  uint8_t _pad2[4];
  double q[H1B_NM];
  double kp[H1B_NM];
  double kd[H1B_NM];
} H1BCmd;  // 528 bytes

// ---------------------------------------------------------------------------------------- lock-free readers
// Seqlock read of the latest snapshot. Returns 0 on success, -1 if no consistent copy within `tries`.
static inline int h1b_snap_read(const H1BShm* s, H1BSnap* out, int tries) {
  for (int k = 0; k < tries; k++) {
    uint64_t a = __atomic_load_n(&s->snap_seq, __ATOMIC_ACQUIRE);
    if (a & 1u) continue;
    __builtin_memcpy(out, (const void*)&s->snap, sizeof(H1BSnap));
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint64_t b = __atomic_load_n(&s->snap_seq, __ATOMIC_RELAXED);
    if (a == b) return 0;
  }
  return -1;
}

// Copy ring entries with idx in [*from, head) (at most the newest H1B_RING, at most `max`). Advances *from past the
// entries returned; entries overwritten while being read are skipped (*n_lost counts every skipped index).
static inline int h1b_ring_read(const H1BShm* s, uint64_t* from, H1BRingEntry* out, int max, uint64_t* n_lost) {
  uint64_t head = __atomic_load_n(&s->ring_head, __ATOMIC_ACQUIRE);
  uint64_t i = *from;
  if (head > i + H1B_RING) { *n_lost += head - H1B_RING - i; i = head - H1B_RING; }
  int n = 0;
  for (; i < head && n < max; i++) {
    const H1BRingEntry* e = &s->ring[i & (H1B_RING - 1)];
    uint64_t a = __atomic_load_n(&e->idx, __ATOMIC_ACQUIRE);
    H1BRingEntry c;
    __builtin_memcpy(&c, (const void*)e, sizeof c);
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint64_t b = __atomic_load_n(&e->idx, __ATOMIC_RELAXED);
    if (a != i || b != i) { (*n_lost)++; continue; }
    out[n++] = c;
  }
  *from = i;
  return n;
}
