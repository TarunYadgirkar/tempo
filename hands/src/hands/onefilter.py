"""One Euro filter, one instance per coordinate.

Casiez, Roussel and Vogel (CHI 2012). A low-pass filter whose cutoff rises
with the speed of the signal, so a joint held still is smoothed hard (the
jitter this whole project exists to remove) while a joint being moved fast is
barely delayed. A fixed low-pass cannot do both: tuned to kill the jitter it
adds lag you can feel on a pinch, and tuned for response it leaves the jitter.

The gesture engine already runs its own One Euro (gesture-engine/src/one_euro.h)
on the joints it receives. Filtering here as well is deliberate: this stage
sees per-landmark depth noise from the LiDAR sample, which is a different and
much coarser noise than what the engine's stage was tuned against.
"""

from __future__ import annotations

import math


class OneEuro:
    """Scalar filter. `min_cutoff` sets the floor; `beta` the speed coupling."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.5, d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_prev: float | None = None
        self._dx_prev = 0.0
        self._t_prev = 0.0

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x: float, t_s: float) -> float:
        if self._x_prev is None:
            self._x_prev = x
            self._t_prev = t_s
            return x

        dt = t_s - self._t_prev
        if dt <= 0.0:
            dt = 1.0 / 30.0
        self._t_prev = t_s

        dx = (x - self._x_prev) / dt
        dx_hat = self._alpha(self.d_cutoff, dt) * dx + (
            1.0 - self._alpha(self.d_cutoff, dt)
        ) * self._dx_prev
        self._dx_prev = dx_hat

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        return x_hat

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = 0.0


class JointFilter:
    """21 landmarks x 3 axes of OneEuro, reset together when tracking drops."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.5, joints: int = 21):
        self._f = [
            [OneEuro(min_cutoff, beta) for _ in range(3)] for _ in range(joints)
        ]

    def apply(self, joints, t_s: float):
        """`joints` is an (n, 3) array; returns the filtered copy."""
        out = joints.copy()
        for j in range(out.shape[0]):
            for axis in range(3):
                out[j, axis] = self._f[j][axis](float(joints[j, axis]), t_s)
        return out

    def reset(self) -> None:
        for joint in self._f:
            for f in joint:
                f.reset()
