"""Same-PC file protocol for gradual support release in the DDS simulator.

An empty file means full support; no file means no support. A release command
uses the shared monotonic clock so MuJoCo can interpolate at each physics step.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile


def release_scale(elapsed_s: float, duration_s: float) -> float:
    """Smoothstep from full spring/damping strength to zero, with flat ends."""
    if duration_s <= 0:
        return 0.0
    x = min(1.0, max(0.0, elapsed_s / duration_s))
    return 1.0 - x * x * (3.0 - 2.0 * x)


def write_band_command(path, *, release_start=None, release_s=0.0):
    """Atomically replace the command; default restores full support immediately."""
    path = Path(path)
    content = '' if release_start is None else json.dumps({
        'release_start_monotonic_s': float(release_start),
        'release_duration_s': float(release_s),
    })
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                         prefix=f'.{path.name}.', delete=False) as f:
            temporary = Path(f.name)
            f.write(content)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class BandSupportFile:
    def __init__(self, path):
        self.path = Path(path) if path else None
        self.signature = None
        self.release = None
        self.support_generation = 0

    def scale(self, now: float) -> float:
        if self.path is None:
            return 0.0
        try:
            stat = self.path.stat()
            signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            if signature != self.signature:
                content = self.path.read_text().strip()
                self.release = None
                try:
                    command = json.loads(content) if content else {}
                    start = float(command['release_start_monotonic_s'])
                    duration = float(command['release_duration_s'])
                    if math.isfinite(start) and math.isfinite(duration) and duration >= 0:
                        self.release = (start, duration)
                except (ValueError, KeyError, TypeError):
                    pass  # Empty/legacy/malformed files retain full support.
                if self.release is None:
                    # Detect re-hoisting even when DAMP cancels a partial release.
                    self.support_generation += 1
                self.signature = signature
        except FileNotFoundError:
            self.signature = self.release = None
            return 0.0
        if self.release is None:
            return 1.0
        start, duration = self.release
        return release_scale(now - start, duration)
