"""Three-clip selection using the runners' existing supported PD transitions.

This module chooses references; it does not change gains, reference coordinates,
policy inference or safety thresholds. Both DDS runners use the same routing.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

# 定义按键组合与动作映射，L1+A/X/Y 分别对应三种动作选择
MOTION_BUTTONS = {"default": ("L1", "A"), "walk": ("L1", "X"), "dance": ("L1", "Y")}


def selection_combos(combo_mask):
    return {**{f"select_{name}": combo_mask(*keys) for name, keys in MOTION_BUTTONS.items()},
            "policy": combo_mask("R1", "B"),
            "start": combo_mask("start"), "damp": combo_mask("L2", "B")}


def validate_selection_args(args, parser):
    if not args.motion_select:
        return
    try:
        MotionSelection.from_paths(args.clip)
    except ValueError as exc:
        parser.error(str(exc))
    if args.auto_sequence or args.end_hold_legacy:
        parser.error("--motion-select requires manual Start playback and STANDBY_POST; omit --auto-sequence/--end-hold-legacy")


@dataclass
class MotionSelection:
    indices: dict[str, int]
    pending: int | None = None
    preparing: bool = False
    ready: bool = False
    last_hold_notice: float = float("-inf")

    @classmethod
    def from_paths(cls, paths):
        paths = [Path(p) for p in paths]
        if len(paths) != 3 or len({p.resolve() for p in paths}) != 3:
            raise ValueError("--motion-select requires exactly three distinct clip files in --clip; "
                             "custom filenames map in order to L1+A, L1+X, L1+Y")
        names = [p.stem for p in paths]
        # Keep the original filename-based mapping for existing commands in any order.
        if set(names) == set(MOTION_BUTTONS):
            return cls({name: i for i, name in enumerate(names)})
        # Other datasets use the three button slots in command-line order.
        return cls({role: i for i, role in enumerate(MOTION_BUTTONS)})

    def button_summary(self, paths):
        return " | ".join(f"{'+'.join(keys)} = {Path(paths[self.indices[role]]).name}"
                          for role, keys in MOTION_BUTTONS.items())


class MotionSelectionMixin:
    """Small adapter to Runner: event routing plus the existing END_HOLD setup."""

    @property
    def policy_button(self):
        return "R1+B" if self.motion_selection is not None else "R1+A"

    @property
    def play_button(self):
        return "Start" if self.motion_selection is not None else "R1+X"

    def motion_events(self, state, t_in, st, events):
        selection = self.motion_selection
        if selection is None:
            return events
        if self.state == "DAMP" or self.damp_req is not None:
            selection.pending = None
            selection.preparing = selection.ready = False
            return []
        # A bridge thermal stop must finish under the original protection logic.
        if getattr(self, "thermal", None) is not None:
            selection.pending = None
            selection.preparing = selection.ready = False
            if events:
                self.event("REFUSED", "motion selection/playback unavailable after thermal stop; use L2+B")
            return []

        # Re-evaluate the original gate; READY never bypasses a fresh Start check.
        if state == "STANDBY_PRE":
            ok, why = self.standby_allowed(t_in, st)
            if ok and not selection.ready:
                selection.preparing = False
                self.event("READY", f"{self.clips[self.clip_idx].name}: 准备完成，可以按 Start 播放")
            elif not ok and selection.ready:
                self.event("WAIT", f"Start 暂不可用: {why}")
            selection.ready = ok
        else:
            selection.ready = False

        choices = [ev for ev in events if ev.startswith("select_")]
        if len(choices) > 1:
            self.event("REFUSED", "multiple motion buttons pressed; select one motion")
            events = []
        elif choices:
            target = selection.indices[choices[0][len("select_"):]]
            events = []  # Simultaneous takeover/Start requests are never queued.
            if selection.preparing or state in ("END_HOLD", "INTERP", "HOLD"):
                self.event("INFO", "motion preparation in progress; selection ignored")
            elif target == self.clip_idx and state in ("STANDBY_PRE", "POLICY"):
                self.event("INFO", f"{self.clips[target].name} already selected/playing; no restart")
            elif state in ("PASSIVE", "STANDBY_PRE", "STANDBY_POST", "POLICY"):
                selection.pending = target
                suffix = "; wait for the current motion to finish" if state == "POLICY" else ""
                self.event("SELECT", f"{self.clips[self.clip_idx].name} -> {self.clips[target].name}{suffix}")

        if selection.pending is not None and state in ("PASSIVE", "STANDBY_PRE", "STANDBY_POST"):
            return ["select_motion"]
        if selection.preparing and state == "HOLD":
            now = time.monotonic()
            if now - selection.last_hold_notice >= self.a.status_s:
                selection.last_hold_notice = now
                self.event("WAIT", "固定 PD 保持；按 R1+B 检查并接管策略；尚未进入策略控制")
        if "policy" in events:
            # The Runner applies the original HOLD gate to this one request.
            # A simultaneous Start is discarded even if takeover succeeds.
            return ["policy"]
        return [ev for ev in events if ev == "start"]

    def begin_selected_motion(self, st):
        """Return an interpolation origin only when starting from PASSIVE."""
        selection = self.motion_selection
        if self.state == "DAMP" or self.damp_req is not None or selection.pending is None:
            return None
        target = selection.pending
        selection.pending = None
        selection.preparing, selection.ready = True, False
        selection.last_hold_notice = float("-inf")
        self.confirm = None
        if self.state == "PASSIVE":
            self.clip_idx = target
            self.set_state("INTERP", f"selected {self.clips[target].name}; hoist must carry during fixed PD preparation")
            return st.q.copy()
        pre = self.state == "STANDBY_PRE"
        # Identical gains/target blend to the old confirmed L2+up transition.
        self.end = dict(q0=(self.last_cmd_q if self.last_cmd_q is not None else st.q).copy(),
                        f_end=(self.f0s if pre else self.fends)[self.clip_idx],
                        then_interp=True, target_idx=target)
        self.set_state("END_HOLD", f"selected {self.clips[target].name}: {self.a.end_blend_s} s PD blend, then INTERP")
        self.event("WARN", "fixed PD transition: external support must carry the robot")
        return None
