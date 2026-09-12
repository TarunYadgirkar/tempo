from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

from . import perception
from .actions import Executor
from .brain import Brain
from .shell import Shell

ENV_FILE = Path(__file__).resolve().parents[2] / ".env"


def _speak(text: str) -> None:
    subprocess.Popen(["say", "-r", "190", text])


def _handle(shell: Shell, brain: Brain, request: str, audio: bytes | None, speak: bool) -> None:
    t0 = time.time()
    snap = perception.take(shell)
    t1 = time.time()
    calls, text = brain.decide(request, snap, audio)
    t2 = time.time()
    outcome = Executor(shell, snap).run(calls)
    for line in outcome.log:
        print("  ·", line)
    reply = outcome.spoken or text.strip()
    if reply:
        print("tempo:", reply)
        if speak:
            _speak(reply)
    print(f"  [snapshot {t1 - t0:.2f}s · gemini {t2 - t1:.2f}s · {len(calls)} actions]")


def cmd_ask(args: argparse.Namespace) -> None:
    shell = Shell()
    brain = Brain(args.model)
    _handle(shell, brain, " ".join(args.words), None, args.speak)


def cmd_listen(args: argparse.Namespace) -> None:
    from .voice import record_until_enter

    shell = Shell()
    brain = Brain(args.model)
    while True:
        input("press Enter to talk (Ctrl-C to quit) ")
        wav = record_until_enter()
        _handle(shell, brain, "", wav, True)


def cmd_scene(args: argparse.Namespace) -> None:
    snap = perception.take(Shell())
    Path("last-view.jpg").write_bytes(snap.png)
    print(snap.scene_text())
    print("view saved to last-view.jpg")


def cmd_eval(args: argparse.Namespace) -> None:
    from .eval import run, summarize

    for condition in args.conditions:
        trials = run(condition, args.limit)
        print(condition, summarize(trials))


def main() -> None:
    load_dotenv(ENV_FILE)
    p = argparse.ArgumentParser(prog="tempo")
    p.add_argument("--model")
    p.add_argument("--speak", action="store_true")
    sub = p.add_subparsers(required=True)
    a = sub.add_parser("ask", help="one typed request")
    a.add_argument("words", nargs="+")
    a.set_defaults(fn=cmd_ask)
    sub.add_parser("listen", help="push-to-talk loop").set_defaults(fn=cmd_listen)
    sub.add_parser("scene", help="dump what the agent would see").set_defaults(fn=cmd_scene)
    e = sub.add_parser("eval", help="run the fixed request set")
    e.add_argument("conditions", nargs="*", default=["geometry", "pixels"])
    e.add_argument("--limit", type=int)
    e.set_defaults(fn=cmd_eval)
    args = p.parse_args()
    try:
        args.fn(args)
    except KeyboardInterrupt:
        sys.exit(0)
