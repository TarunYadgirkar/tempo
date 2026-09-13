"""The shell's control socket, from Python.

Line protocol: one request line in, one `ok ...` / `err <code> <detail>` line
back. The request line is capped at CTL_CONN_INPUT_MAX (~1.1 kB) by the
vendored wire layer, which is why `inject_hands` sends one hand per message
when two are in view — the shell merges them by chirality.
"""

from __future__ import annotations

import json
import os
import socket

# Leaves room for the verb, the wrapper and the newline inside the ~1.1 kB the
# wire layer will assemble before it declares the peer abusive.
MAX_LINE = 1000
# Millimetre resolution. The LiDAR sample feeding these is noisier than that by
# an order of magnitude, so the digits beyond it are only wire bytes, and the
# bytes are what the line cap is made of.
JOINT_DECIMALS = 4


class ShellError(RuntimeError):
    pass


class ShellControl:
    def __init__(self, sock_path: str | None = None, timeout_s: float = 2.0):
        self.path = sock_path or os.environ.get("SPATIAL_OS_SOCK") or _default_sock()
        if not self.path:
            raise ShellError(
                "no control socket: pass --sock or set SPATIAL_OS_SOCK"
            )
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.settimeout(timeout_s)
        try:
            self._sock.connect(self.path)
        except OSError as exc:
            raise ShellError(f"cannot connect to {self.path}: {exc}") from exc
        self._buf = b""

    def close(self) -> None:
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def request(self, line: str) -> str:
        self._sock.sendall(line.encode() + b"\n")
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ShellError("control socket closed mid-request")
            self._buf += chunk
        reply, _, self._buf = self._buf.partition(b"\n")
        return reply.decode(errors="replace")

    # -- verbs -------------------------------------------------------------

    def version(self) -> str:
        return self.request("version")

    def hands_status(self) -> dict:
        """`hands status` -> {"overlay", "source", "age_ms", "e2e_ms"}.

        `e2e_ms` is -1 when the shell holds no injection carrying a frame
        stamp, so it is signed and the parse has to admit a leading minus.
        """
        reply = self.request("hands status")
        out: dict[str, str | int] = {}
        for token in reply.removeprefix("ok ").split():
            key, _, value = token.partition("=")
            out[key] = int(value) if value.lstrip("-").isdigit() else value
        return out

    def hands_dump(self) -> dict:
        """`hands dump` -> the joints the gesture engine is seeing right now."""
        reply = self.request("hands dump")
        if not reply.startswith("ok "):
            raise ShellError(reply)
        return json.loads(reply[3:])

    def frame_export_on(self, directory: str, raw: bool = False) -> str:
        """Turn the export on. `raw` publishes latest.rgb (decoded pixels,
        downscaled by the shell) instead of a JPEG this process would have to
        decode again."""
        reply = self.request(
            f"frame-export on {directory}" + (" --raw" if raw else "")
        )
        if not reply.startswith("ok "):
            raise ShellError(reply)
        return reply

    def frame_export_off(self) -> str:
        return self.request("frame-export off")

    def inject_hands(
        self, t_ms: int, hands: list[dict], frame_t_ns: int = 0
    ) -> None:
        """Send `hands-inject`, splitting when two hands overflow the line.

        `hands` entries are {"chirality", "confidence", "joints"} with joints
        as a (21, 3) array of scene-frame metres in MediaPipe landmark order.
        An empty list is the explicit "no hands in view", which retracts both.

        `frame_t_ns` is the `export_ns` of the frame these joints came from,
        echoed back so `hands status` can report export -> injection on the
        shell's own clock. 0 omits it, and the shell then reports no latency
        rather than an invented one.
        """
        for payload in self._payloads(t_ms, hands, frame_t_ns):
            reply = self.request(f"hands-inject {payload}")
            if not reply.startswith("ok"):
                raise ShellError(reply)

    @staticmethod
    def _payloads(t_ms: int, hands: list[dict], frame_t_ns: int = 0) -> list[str]:
        encoded = [_encode_hand(h) for h in hands]
        whole = _wrap(t_ms, encoded, frame_t_ns)
        if len(whole) + len("hands-inject ") <= MAX_LINE or not encoded:
            return [whole]
        return [_wrap(t_ms, [one], frame_t_ns) for one in encoded]


def _encode_hand(hand: dict) -> str:
    joints = ",".join(
        "[{:.{d}f},{:.{d}f},{:.{d}f}]".format(*(float(c) for c in j), d=JOINT_DECIMALS)
        for j in hand["joints"]
    )
    return (
        '{{"chirality":"{c}","confidence":{k:.3f},"joints":[{j}]}}'.format(
            c=hand["chirality"], k=float(hand.get("confidence", 1.0)), j=joints
        )
    )


def _wrap(t_ms: int, encoded: list[str], frame_t_ns: int = 0) -> str:
    stamp = f',"frame_t_ns":{int(frame_t_ns)}' if frame_t_ns else ""
    return '{{"t":{t}{s},"hands":[{h}]}}'.format(
        t=int(t_ms), s=stamp, h=",".join(encoded)
    )


def _default_sock() -> str:
    tmp = os.environ.get("TMPDIR", "/tmp").rstrip("/")
    return f"{tmp}/spatial-os.sock"
