"""Line-based client for the Spatula/Tempo compositor control socket."""
from __future__ import annotations

import json
import os
import socket
import time
from dataclasses import dataclass
from typing import Any, Iterator


class ShellError(RuntimeError):
    pass


def default_sock_path() -> str:
    return os.environ.get("SPATIAL_OS_SOCK") or os.path.join(os.environ["TMPDIR"], "spatial-os.sock")


@dataclass(frozen=True)
class Reply:
    raw: str

    @property
    def ok(self) -> bool:
        return self.raw.startswith("ok")

    @property
    def body(self) -> str:
        return self.raw[2:].strip() if self.ok else self.raw

    def json(self) -> Any:
        return json.loads(self.body)

    def kv(self) -> dict[str, str]:
        return dict(part.split("=", 1) for part in self.body.split() if "=" in part)


class Shell:
    def __init__(self, path: str | None = None) -> None:
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.connect(path or default_sock_path())
        self._io = self._sock.makefile("rw", encoding="utf-8")

    def close(self) -> None:
        self._io.close()
        self._sock.close()

    def send(self, line: str) -> Reply:
        self._io.write(line + "\n")
        self._io.flush()
        reply = Reply(self._io.readline().rstrip("\n"))
        if not reply.ok:
            raise ShellError(f"{line!r} -> {reply.raw}")
        return reply

    def head_pose(self) -> dict[str, Any]:
        return self.send("head-pose").json()

    def planes(self) -> list[dict[str, Any]]:
        return self.send("list-planes").json()["planes"]

    def windows(self) -> list[dict[str, Any]]:
        return self.send("list-windows").json()["windows"]

    def stats(self) -> dict[str, str]:
        return self.send("stats").kv()

    def screenshot(self, path: str | None = None) -> str:
        reply = self.send(f"screenshot {path}" if path else "screenshot")
        return reply.kv()["path"]

    def launch_card(self, title: str) -> int:
        before = {w["handle"] for w in self.windows()}
        self.send(f"launch {title}")
        new = [w["handle"] for w in self.windows() if w["handle"] not in before]
        if not new:
            raise ShellError("launch produced no window")
        return new[0]

    def note(self, title: str, body: str, accent: bool = False) -> int:
        payload = json.dumps({"title": title, "body": body, "accent": accent})
        return int(self.send(f"note {payload}").kv()["handle"])

    def aim(self) -> dict[str, Any]:
        return self.send("aim").json()

    def layout_save(self, name: str) -> str:
        return self.send(f"layout save {name}").body

    def layout_load(self, name: str) -> str:
        return self.send(f"layout load {name}").body

    def launch_app(self, target: str, wait_s: float = 2.0) -> int:
        before = {w["handle"] for w in self.windows()}
        self.send(f"launch-app {target}")
        deadline = time.time() + wait_s
        while time.time() < deadline:
            new = [w["handle"] for w in self.windows() if w["handle"] not in before]
            if new:
                return new[0]
            time.sleep(0.1)
        raise ShellError(f"launch-app {target!r} produced no window")

    def move(self, handle: int, pos: tuple[float, float, float]) -> None:
        x, y, z = pos
        self.send(f"move {handle} {x:.4f} {y:.4f} {z:.4f}")

    def anchor(self, handle: int, target: str) -> str:
        return self.send(f"anchor {handle} {target}").body

    def close_window(self, handle: int) -> None:
        self.send(f"close {handle}")

    def focus(self, handle: int) -> None:
        self.send(f"focus {handle}")

    def resize(self, handle: int, w: int, h: int) -> None:
        self.send(f"resize {handle} {w} {h}")

    def gather(self) -> None:
        self.send("gather-panels")

    def events(self) -> Iterator[str]:
        self.send("subscribe")
        while True:
            line = self._io.readline()
            if not line:
                return
            line = line.rstrip("\n")
            if line.startswith("event "):
                yield line[6:]
