// libh1bridge_client.so: lock-free shm readers for the Python runner (ctypes). Plain C ABI, no DDS.
#include <fcntl.h>
#include <stddef.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "h1_bridge_shm.h"

extern "C" {

// Map the bridge's shm read-only. NULL if absent / not yet initialised / wrong version.
void* h1b_open(const char* name) {
  int fd = shm_open(name, O_RDONLY, 0);
  if (fd < 0) return nullptr;
  void* p = mmap(nullptr, sizeof(H1BShm), PROT_READ, MAP_SHARED, fd, 0);
  close(fd);
  if (p == MAP_FAILED) return nullptr;
  const H1BShm* s = static_cast<const H1BShm*>(p);
  if (__atomic_load_n(&s->magic, __ATOMIC_ACQUIRE) != H1B_MAGIC || s->version != H1B_VERSION) {
    munmap(p, sizeof(H1BShm));
    return nullptr;
  }
  return p;
}

void h1b_close(void* p) {
  if (p) munmap(p, sizeof(H1BShm));
}

// sizes: snap, ring entry, status, cmd, shm
void h1b_sizes(long long* out) {
  out[0] = sizeof(H1BSnap);
  out[1] = sizeof(H1BRingEntry);
  out[2] = sizeof(H1BStatus);
  out[3] = sizeof(H1BCmd);
  out[4] = sizeof(H1BShm);
}

// field offsets, in the order of the Python dtypes (checked at startup)
int h1b_snap_offsets(long long* out) {
  int k = 0;
  out[k++] = offsetof(H1BSnap, t_rx);
  out[k++] = offsetof(H1BSnap, t_tick_change);
  out[k++] = offsetof(H1BSnap, n_rx);
  out[k++] = offsetof(H1BSnap, n_crc_bad);
  out[k++] = offsetof(H1BSnap, tick);
  out[k++] = offsetof(H1BSnap, tick_moving);
  out[k++] = offsetof(H1BSnap, reserved_mode);
  out[k++] = offsetof(H1BSnap, reserved_control);
  out[k++] = offsetof(H1BSnap, q);
  out[k++] = offsetof(H1BSnap, dq);
  out[k++] = offsetof(H1BSnap, tau);
  out[k++] = offsetof(H1BSnap, temp);
  out[k++] = offsetof(H1BSnap, motorstate);
  out[k++] = offsetof(H1BSnap, _pad0);
  out[k++] = offsetof(H1BSnap, quat);
  out[k++] = offsetof(H1BSnap, gyro);
  out[k++] = offsetof(H1BSnap, acc);
  out[k++] = offsetof(H1BSnap, rpy);
  out[k++] = offsetof(H1BSnap, wireless_remote);
  return k;
}

int h1b_status_offsets(long long* out) {
  int k = 0;
  out[k++] = offsetof(H1BStatus, t_hb);
  out[k++] = offsetof(H1BStatus, n_pub);
  out[k++] = offsetof(H1BStatus, n_pub_fail);
  out[k++] = offsetof(H1BStatus, fail_consec);
  out[k++] = offsetof(H1BStatus, n_cmd_rx);
  out[k++] = offsetof(H1BStatus, t_last_cmd);
  out[k++] = offsetof(H1BStatus, lc_count);
  out[k++] = offsetof(H1BStatus, lc_t);
  out[k++] = offsetof(H1BStatus, ready);
  out[k++] = offsetof(H1BStatus, pub_started);
  out[k++] = offsetof(H1BStatus, wd_fired);
  out[k++] = offsetof(H1BStatus, quit);
  out[k++] = offsetof(H1BStatus, bridge_pid);
  out[k++] = offsetof(H1BStatus, _pad);
  out[k++] = offsetof(H1BStatus, wd_s);
  out[k++] = offsetof(H1BStatus, pub_gap_max);
  out[k++] = offsetof(H1BStatus, n_gap_late);
  out[k++] = offsetof(H1BStatus, pub_write_max);
  return k;
}

int h1b_read_snap(const void* p, void* out) { return h1b_snap_read(static_cast<const H1BShm*>(p), static_cast<H1BSnap*>(out), 1000); }

int h1b_read_ring(const void* p, unsigned long long* from, void* out, int max, unsigned long long* n_lost) {
  uint64_t f = *from, l = *n_lost;
  int n = h1b_ring_read(static_cast<const H1BShm*>(p), &f, static_cast<H1BRingEntry*>(out), max, &l);
  *from = f;
  *n_lost = l;
  return n;
}

// Status fields are independent counters: one acquire fence, then a plain copy.
void h1b_read_status(const void* p, void* out) {
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  memcpy(out, (const void*)&static_cast<const H1BShm*>(p)->status, sizeof(H1BStatus));
}
}
