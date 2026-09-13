"""The spatial actions the agent may take, as Gemini tool declarations + an executor."""
from __future__ import annotations

from dataclasses import dataclass, field

from google.genai import types

from . import spatial
from .memory import Memory, relative_direction
from .perception import Snapshot
from .shell import Shell, ShellError

WHERE = ["in_front", "left", "right", "on_table", "on_wall", "where_looking", "where_pointing", "near_object"]
PANEL_GAP_M = 0.45
OBJECT_STANDOFF_M = 0.25  # how far in front of a detected object its panel floats


def _where_schema(desc: str) -> types.Schema:
    return types.Schema(type=types.Type.STRING, enum=WHERE, description=desc)


DECLARATIONS = [
    types.FunctionDeclaration(
        name="place_note",
        description="Create a note card in the room. Use for reminders, labels, lists, answers the wearer should keep seeing.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "title": types.Schema(type=types.Type.STRING, description="Two to four word title."),
                "body": types.Schema(type=types.Type.STRING, description="The note text, one to three short sentences."),
                "where": _where_schema("Where to put it. Use near_object when the request names a physical object the scene's objects list knows; where_pointing when the wearer's hand is pointing at something; where_looking for 'here' or 'there'."),
                "object": types.Schema(type=types.Type.STRING, description="Required when where is near_object: the object to put it beside, e.g. lamp, monitor, plant. Must be one of the labels in the scene's objects list."),
            },
            required=["title", "body", "where"],
        ),
    ),
    types.FunctionDeclaration(
        name="open_app",
        description="Open a Mac app as a floating panel in the room.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "app": types.Schema(type=types.Type.STRING, description="App name as the user says it, e.g. Safari, Notes, Terminal, Spotify."),
                "where": _where_schema("Where to put the panel. near_object puts it beside a physical object from the scene's objects list."),
                "object": types.Schema(type=types.Type.STRING, description="Required when where is near_object: the object to put it beside, e.g. lamp, monitor, plant. Must be one of the labels in the scene's objects list."),
            },
            required=["app", "where"],
        ),
    ),
    types.FunctionDeclaration(
        name="move_panel",
        description="Move an existing panel (by handle from the scene) somewhere else.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "handle": types.Schema(type=types.Type.INTEGER),
                "where": _where_schema("Destination."),
            },
            required=["handle", "where"],
        ),
    ),
    types.FunctionDeclaration(
        name="close_panel",
        description="Close a panel by handle.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"handle": types.Schema(type=types.Type.INTEGER)},
            required=["handle"],
        ),
    ),
    types.FunctionDeclaration(
        name="gather_panels",
        description="Bring every panel back in front of the wearer. Use when they say they lost their windows.",
        parameters=types.Schema(type=types.Type.OBJECT, properties={}),
    ),
    types.FunctionDeclaration(
        name="remember_place",
        description="Save a named spot in the room: where the wearer is pointing, or looking, or a panel's location. Use when they say 'remember this is X' or 'this is where I keep X'.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "label": types.Schema(type=types.Type.STRING, description="Short name, e.g. 'charger', 'keys', 'reading spot'."),
                "where": _where_schema("where_pointing if their hand is visible, else where_looking."),
            },
            required=["label", "where"],
        ),
    ),
    types.FunctionDeclaration(
        name="recall_place",
        description="Find a remembered spot by name, tell the wearer where it is relative to them, and optionally drop a marker note there.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "label": types.Schema(type=types.Type.STRING),
                "mark": types.Schema(type=types.Type.BOOLEAN, description="Put a small marker note at the spot."),
            },
            required=["label"],
        ),
    ),
    types.FunctionDeclaration(
        name="save_layout",
        description="Save every panel's position under a name, e.g. 'desk' or 'bed', so it can be restored later.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"name": types.Schema(type=types.Type.STRING, description="Letters, digits, dashes only.")},
            required=["name"],
        ),
    ),
    types.FunctionDeclaration(
        name="restore_layout",
        description="Bring back a saved panel layout by name.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"name": types.Schema(type=types.Type.STRING)},
            required=["name"],
        ),
    ),
    types.FunctionDeclaration(
        name="say",
        description="Speak a short reply to the wearer. Always call this once, last, with one sentence.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"text": types.Schema(type=types.Type.STRING)},
            required=["text"],
        ),
    ),
]

def _offset_props() -> dict[str, types.Schema]:
    return {
        "forward_m": types.Schema(type=types.Type.NUMBER, description="Metres ahead of the wearer's head, along their gaze. Estimate from the image."),
        "right_m": types.Schema(type=types.Type.NUMBER, description="Metres to the wearer's right (negative = left)."),
        "up_m": types.Schema(type=types.Type.NUMBER, description="Metres above eye level (negative = below)."),
    }


PIXELS_DECLARATIONS = [
    types.FunctionDeclaration(
        name="place_note_at",
        description="Create a note card at a position you estimate from the image alone, relative to the wearer's head.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "title": types.Schema(type=types.Type.STRING),
                "body": types.Schema(type=types.Type.STRING),
                **_offset_props(),
            },
            required=["title", "body", "forward_m", "right_m", "up_m"],
        ),
    ),
    types.FunctionDeclaration(
        name="open_app_at",
        description="Open a Mac app panel at a position you estimate from the image alone, relative to the wearer's head.",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"app": types.Schema(type=types.Type.STRING), **_offset_props()},
            required=["app", "forward_m", "right_m", "up_m"],
        ),
    ),
    DECLARATIONS[-1],
]


APP_ALIASES = {
    "safari": "com.apple.Safari",
    "notes": "com.apple.Notes",
    "terminal": "com.apple.Terminal",
    "finder": "com.apple.finder",
    "messages": "com.apple.MobileSMS",
    "music": "com.apple.Music",
    "spotify": "com.spotify.client",
    "calendar": "com.apple.iCal",
    "chrome": "com.google.Chrome",
    "mail": "com.apple.mail",
    "reminders": "com.apple.reminders",
    "vscode": "com.microsoft.VSCode",
    "code": "com.microsoft.VSCode",
}


@dataclass
class Outcome:
    log: list[str] = field(default_factory=list)
    spoken: str | None = None


class Executor:
    def __init__(self, shell: Shell, snap: Snapshot, memory: Memory | None = None) -> None:
        self.shell = shell
        self.snap = snap
        self.memory = memory or Memory()
        self.placed = 0

    def run(self, calls: list[types.FunctionCall]) -> Outcome:
        out = Outcome()
        for call in calls:
            handler = getattr(self, f"_do_{call.name}", None)
            if handler is None:
                out.log.append(f"unknown action {call.name}")
                continue
            try:
                msg = handler(out, **(call.args or {}))
            except ShellError as e:
                msg = f"{call.name} failed: {e}"
            out.log.append(msg)
        return out

    def _target(self, where: str) -> spatial.Vec3:
        head = self.snap.head
        side = self.placed * PANEL_GAP_M
        self.placed += 1
        match where:
            case "left":
                return spatial.in_front(head, 0.8, right=-0.5 - side)
            case "right":
                return spatial.in_front(head, 0.8, right=0.5 + side)
            case "on_table":
                hit = spatial.gaze_hit(head, self.snap.planes, {"table"})
                return spatial.add(hit, (0, 0.12, 0)) if hit else spatial.in_front(head, 0.7, up=-0.25, right=side)
            case "on_wall":
                hit = spatial.gaze_hit(head, self.snap.planes, {"wall"})
                return hit or spatial.in_front(head, 1.6, right=side)
            case "where_looking":
                hit = spatial.gaze_hit(head, self.snap.planes, {"wall", "table", "floor"})
                return hit or spatial.in_front(head, 1.2, right=side)
            case "where_pointing":
                hit = self.snap.aim.get("hit")
                if hit:
                    return spatial.add(hit, spatial.scale(spatial.normalize(spatial.sub(head["scene_pos"], hit)), 0.08))
                return self._target("where_looking")
            case _:
                return spatial.in_front(head, 0.9, right=side)

    SURFACE_WHERE = {"on_table": "horizontal", "on_wall": "vertical", "where_looking": None, "where_pointing": None}

    def _surface_hit(self, where: str) -> dict | None:
        """Depth-map hit for surface placements, when the shell can cast and the hit kind matches."""
        if where not in self.SURFACE_WHERE:
            return None
        if where == "where_pointing" and self.snap.aim.get("ray_origin"):
            hit = self.shell.cast(self.snap.aim["ray_origin"], self.snap.aim["ray_dir"])
        else:
            hit = self.shell.cast()
        if not hit or not hit.get("hit"):
            return None
        want = self.SURFACE_WHERE[where]
        if want and hit.get("kind") != want:
            return None
        return hit

    def _object_pose(self, name: str) -> tuple[spatial.Vec3, spatial.Quat] | None:
        """Standing 25 cm in front of a tracked object, upright, square to the viewer."""
        from . import objects

        viewer = self.snap.head["scene_pos"]
        found = objects.nearest(name, viewer)
        if not found:
            return None
        pos = found["position_m"]
        toward_viewer = spatial.normalize(spatial.sub(viewer, pos))
        return spatial.surface_pose(pos, toward_viewer, viewer, lift_m=OBJECT_STANDOFF_M)

    def _place(self, handle: int, where: str, obj: str | None = None) -> None:
        if where == "near_object" and obj:
            pose = self._object_pose(obj)
            if pose and self.shell.pose(handle, *pose):
                return
        hit = self._surface_hit(where)
        if hit:
            pos, quat = spatial.surface_pose(hit["hit"], hit["normal"], self.snap.head["scene_pos"])
            if self.shell.pose(handle, pos, quat):
                return
        self.shell.move(handle, self._target(where))
        if where == "on_table":
            self.shell.anchor(handle, "closest-horizontal")
        elif where in ("on_wall", "where_looking", "where_pointing"):
            self.shell.anchor(handle, "closest-wall")

    def _do_place_note(self, out: Outcome, title: str, body: str, where: str, object: str | None = None) -> str:
        handle = self.shell.note(title, body, accent=True)
        self._place(handle, where, object)
        return f"note #{handle} {title!r} -> {where}{f' ({object})' if object else ''}"

    def _do_open_app(self, out: Outcome, app: str, where: str, object: str | None = None) -> str:
        target = APP_ALIASES.get(app.lower().strip(), app)
        handle = self.shell.launch_app(target)
        self._place(handle, where, object)
        return f"app {app} #{handle} -> {where}{f' ({object})' if object else ''}"

    def _do_move_panel(self, out: Outcome, handle: int, where: str) -> str:
        self._place(int(handle), where)
        return f"panel #{handle} -> {where}"

    def _do_close_panel(self, out: Outcome, handle: int) -> str:
        self.shell.close_window(int(handle))
        return f"closed #{handle}"

    def _do_gather_panels(self, out: Outcome) -> str:
        self.shell.gather()
        return "gathered"

    def _offset(self, forward_m: float, right_m: float, up_m: float) -> spatial.Vec3:
        return spatial.in_front(self.snap.head, float(forward_m), right=float(right_m), up=float(up_m))

    def _do_place_note_at(self, out: Outcome, title: str, body: str, forward_m: float, right_m: float, up_m: float) -> str:
        handle = self.shell.note(title, body, accent=True)
        self.shell.move(handle, self._offset(forward_m, right_m, up_m))
        return f"note #{handle} {title!r} @ f={forward_m} r={right_m} u={up_m}"

    def _do_open_app_at(self, out: Outcome, app: str, forward_m: float, right_m: float, up_m: float) -> str:
        handle = self.shell.launch_app(APP_ALIASES.get(app.lower().strip(), app))
        self.shell.move(handle, self._offset(forward_m, right_m, up_m))
        return f"app {app} #{handle} @ f={forward_m} r={right_m} u={up_m}"

    def _spot(self, where: str) -> tuple[spatial.Vec3, str]:
        hit = self._surface_hit(where)
        if hit:
            return tuple(hit["hit"]), hit.get("kind", "surface")
        return self._target(where), "air"

    def _do_remember_place(self, out: Outcome, label: str, where: str) -> str:
        pos, kind = self._spot(where)
        self.memory.remember(label, pos, kind)
        return f"remembered {label!r} at {[round(v, 2) for v in pos]} ({kind})"

    def _do_recall_place(self, out: Outcome, label: str, mark: bool = False) -> str:
        place = self.memory.find(label)
        if place is None:
            out.spoken = f"I don't have a spot called {label}."
            return f"no memory for {label!r}"
        phrase = relative_direction(self.snap.head, place.pos)
        if mark:
            handle = self.shell.note(place.label, f"Here: {phrase}", accent=True)
            _, quat = spatial.surface_pose(place.pos, spatial.sub(self.snap.head["scene_pos"], place.pos), self.snap.head["scene_pos"], lift_m=0.0)
            self.shell.pose(handle, spatial.add(place.pos, (0, 0.15, 0)), quat)
        out.spoken = f"{place.label} is {phrase}."
        return f"recalled {label!r}: {phrase}"

    def _do_save_layout(self, out: Outcome, name: str) -> str:
        return "layout " + self.shell.layout_save(name)

    def _do_restore_layout(self, out: Outcome, name: str) -> str:
        return "layout " + self.shell.layout_load(name)

    def _do_say(self, out: Outcome, text: str) -> str:
        out.spoken = text
        return f"say {text!r}"
