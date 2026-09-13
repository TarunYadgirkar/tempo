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
    from .voice import record_until_enter, transcribe

    shell = Shell()
    brain = Brain(args.model)
    while True:
        input("press Enter to talk (Ctrl-C to quit) ")
        text = transcribe(record_until_enter())
        print("you:", text)
        if text:
            _handle(shell, brain, text, None, True)


def cmd_live(args: argparse.Namespace) -> None:
    from .live import Live

    Live(args.model, speak=not args.quiet).run()


def cmd_scene(args: argparse.Namespace) -> None:
    snap = perception.take(Shell())
    Path("last-view.jpg").write_bytes(snap.png)
    print(snap.scene_text())
    print("view saved to last-view.jpg")


def cmd_objects(args: argparse.Namespace) -> None:
    from . import objects

    prompts = [w.strip() for w in args.prompts.split(",")] if args.prompts else None

    def once(image: Path, meta: dict, base: Path | None = None) -> None:
        located, ms = objects.process_frame(image, meta, base, args.backend, prompts, args.conf)
        print(f"[{ms:.0f} ms · {len(located)} detections · {args.backend}]")
        for d in sorted(located, key=lambda d: -d["confidence"]):
            pos = " ".join(f"{v:+.2f}" for v in d["position_m"])
            print(f"  · {d['label']:<11} {d['confidence']:.2f}  at [{pos}] m  ~{d['size_m_estimate']:.2f} m  ({d['source']})")
        for t in objects.snapshot():
            if t["unseen_s"] > 1.0:
                print(f"  remembered: {t['label']} at {t['position_m']} ({t['unseen_s']}s ago)")

    if args.frames:
        seen = None
        while True:
            frame = objects.read_export(args.frames)
            stamp = frame[1].get("seq", frame[1].get("t_ns")) if frame else None
            if frame and stamp != seen:
                seen = stamp
                once(frame[0], frame[1], Path(args.frames))
                if args.once:
                    return
            elif not frame:
                print(f"waiting for {args.frames}/latest.json", end="\r")
            time.sleep(0.15)

    if args.image:
        once(*objects.read_pair(args.image, args.meta))
        return

    shell = Shell()
    while True:
        once(Path(shell.screenshot()), {"head": shell.head_pose()})
        if args.once:
            return


def cmd_people(args: argparse.Namespace) -> None:
    from . import people as pp

    if args.action == "list":
        store = pp.People()
        for p in store.people.values():
            said = p.last_said() or ""
            print(f"{p.id}  {p.label:<18} faces={len(p.faces)} voices={len(p.voices)}{'  (you)' if p.is_owner else ''}  {said[:60]}")
        return
    if args.action == "forget":
        print("forgot" if pp.People().forget(args.name) else "no such person", args.name)
        return
    if args.action == "summarize":
        store = pp.People()
        p = store.by_name(args.name or "")
        if not p:
            raise SystemExit(f"no person named {args.name!r}")
        line = pp.summarize(p)
        if line:
            p.summary, p.summarized_count = line, len(p.utterances)
            store.save()
        print(line or "no summary (no key, or nothing said)")
        return
    if args.action == "me":
        p = pp.enroll_owner(args.name, log=print)
        print(f"you are {p.name} ({len(p.voices)} voiceprints)")
        return
    if not args.frames:
        raise SystemExit("people: --frames DIR is required to watch the room")
    pp.PeopleDaemon(args.frames, Shell(), mic=not args.no_mic, log=print).run(once=args.once)



def cmd_eval(args: argparse.Namespace) -> None:
    from .eval import run, summarize

    for condition in args.conditions:
        trials = run(condition, args.limit)
        print(condition, summarize(trials))


def objects_backend() -> str:
    from .objects import DEFAULT_BACKEND

    return DEFAULT_BACKEND


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
    lv = sub.add_parser("live", help="pinch-and-talk daemon")
    lv.add_argument("--quiet", action="store_true")
    lv.set_defaults(fn=cmd_live)
    sub.add_parser("scene", help="dump what the agent would see").set_defaults(fn=cmd_scene)
    o = sub.add_parser("objects", help="detect and track room objects in 3D")
    o.add_argument("image", nargs="?", help="image to run on; omit to pull frames from the live shell")
    o.add_argument("--meta", help="sidecar json with head pose, intrinsics, depth (default: <image>.json)")
    o.add_argument("--frames", help="frame export directory written by the shell (latest.jpg + latest.json)")
    o.add_argument("--once", action="store_true", help="one frame instead of a loop")
    o.add_argument("--backend", default=objects_backend(), choices=["owlv2", "owlvit"], help="owlv2 is more accurate, owlvit is ~7x faster")
    o.add_argument("--prompts", help="comma separated vocabulary (default: common room objects)")
    o.add_argument("--conf", type=float, default=0.25)
    o.set_defaults(fn=cmd_objects)
    pe = sub.add_parser("people", help="faces + voices in the room, with a bubble beside each head")
    pe.add_argument("action", nargs="?", default="watch", choices=["watch", "list", "forget", "me", "summarize"])
    pe.add_argument("name", nargs="?", help="for forget/me")
    pe.add_argument("--frames", help="frame export directory written by the shell")
    pe.add_argument("--no-mic", action="store_true", help="faces only, no listening")
    pe.add_argument("--once", action="store_true")
    pe.set_defaults(fn=cmd_people)
    e = sub.add_parser("eval", help="run the fixed request set")
    e.add_argument("conditions", nargs="*", default=["geometry", "pixels"])
    e.add_argument("--limit", type=int)
    e.set_defaults(fn=cmd_eval)
    args = p.parse_args()
    try:
        args.fn(args)
    except KeyboardInterrupt:
        sys.exit(0)
