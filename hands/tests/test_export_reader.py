"""Reading the shell's export directory, both image formats.

Raw mode is the transport half of the latency fix: the shell writes the
decoded pixels it already has instead of encoding a JPEG for this process to
decode again. The contract is deliberately thin — width x height x 3 bytes,
no header — so the reader's whole job is to refuse anything that is not
exactly that rather than hand the detector a frame with last frame's bottom
rows still in it.
"""

import json

import numpy as np
import pytest
from PIL import Image

from hands.export_reader import ExportReader, read_raw_rgb

W, H = 8, 4


def _pixels() -> np.ndarray:
    return np.arange(H * W * 3, dtype=np.uint8).reshape(H, W, 3)


def _sidecar(image: dict, **extra) -> dict:
    meta = {
        "t_ns": 1_000_000_000,
        "export_ns": 1_757_000_000_000_000_000,
        "seq": 1,
        "image": image,
        "head": {"frame": "scene", "pos": [0.0, 0.0, 0.0], "quat": [0, 0, 0, 1]},
        "intrinsics": {
            "fx": 1440.0,
            "fy": 1440.0,
            "cx": 960.0,
            "cy": 540.0,
            "image_width": 1920.0,
            "image_height": 1080.0,
        },
        "depth": None,
    }
    meta.update(extra)
    return meta


def _write(tmp_path, meta: dict) -> None:
    (tmp_path / "latest.json").write_text(json.dumps(meta))


def test_raw_bytes_read_back_as_the_pixels_that_were_written(tmp_path):
    px = _pixels()
    (tmp_path / "latest.rgb").write_bytes(px.tobytes())
    assert np.array_equal(read_raw_rgb(tmp_path / "latest.rgb", W, H), px)


@pytest.mark.parametrize("short_by", [1, 3, W * 3])
def test_a_short_raw_file_is_refused_rather_than_padded(tmp_path, short_by):
    (tmp_path / "latest.rgb").write_bytes(_pixels().tobytes()[:-short_by])
    assert read_raw_rgb(tmp_path / "latest.rgb", W, H) is None


def test_a_missing_raw_file_is_refused(tmp_path):
    assert read_raw_rgb(tmp_path / "nothing.rgb", W, H) is None


def test_the_reader_takes_a_raw_frame_and_reports_the_export_clock(tmp_path):
    px = _pixels()
    (tmp_path / "latest.rgb").write_bytes(px.tobytes())
    _write(
        tmp_path,
        _sidecar(
            {"width": W, "height": H, "file": "latest.rgb", "format": "rgb8"}
        ),
    )

    frame = ExportReader(tmp_path).read()
    assert frame is not None
    assert np.array_equal(frame.rgb, px)
    assert (frame.width, frame.height) == (W, H)
    # export_ns is this Mac's clock and t_ns the phone's: keeping them apart is
    # what makes the reported latency a latency rather than a clock offset.
    assert frame.export_ns == 1_757_000_000_000_000_000
    assert frame.t_ns == 1_000_000_000


def test_a_jpeg_sidecar_still_decodes(tmp_path):
    Image.fromarray(_pixels()).save(tmp_path / "latest.jpg")
    _write(
        tmp_path,
        _sidecar(
            {"width": W, "height": H, "file": "latest.jpg", "format": "jpeg"}
        ),
    )
    frame = ExportReader(tmp_path).read()
    assert frame is not None
    assert frame.rgb.shape == (H, W, 3)


def test_a_sidecar_from_before_raw_mode_is_read_as_a_jpeg(tmp_path):
    """`format` was added with raw mode, so a sidecar without one can only be
    the JPEG it was before."""
    Image.fromarray(_pixels()).save(tmp_path / "latest.jpg")
    meta = _sidecar({"width": W, "height": H, "file": "latest.jpg"})
    del meta["export_ns"]
    _write(tmp_path, meta)
    frame = ExportReader(tmp_path).read()
    assert frame is not None
    assert frame.rgb.shape == (H, W, 3)
    # No export_ns either — which must read as "unknown", not as 1970.
    assert frame.export_ns == 0


def test_a_raw_file_the_sidecar_disagrees_with_is_dropped(tmp_path):
    """The sidecar is the commit record, so a mismatch is a bug on the writing
    side; taking the file anyway would reshape it into a scrambled frame."""
    (tmp_path / "latest.rgb").write_bytes(_pixels().tobytes())
    _write(
        tmp_path,
        _sidecar(
            {"width": W, "height": H + 1, "file": "latest.rgb", "format": "rgb8"}
        ),
    )
    assert ExportReader(tmp_path).read() is None


def test_a_frame_is_yielded_once_and_the_next_one_is_noticed(tmp_path):
    px = _pixels()
    (tmp_path / "latest.rgb").write_bytes(px.tobytes())
    image = {"width": W, "height": H, "file": "latest.rgb", "format": "rgb8"}
    _write(tmp_path, _sidecar(image))

    reader = ExportReader(tmp_path)
    assert reader.read() is not None
    # Polling is on the sidecar's mtime, so an unchanged directory must cost
    # nothing and yield nothing.
    assert reader.read() is None

    _write(tmp_path, _sidecar(image, seq=2, t_ns=2_000_000_000))
    frame = reader.wait(timeout_s=1.0)
    assert frame is not None and frame.seq == 2


def test_nothing_published_yet_is_not_an_error(tmp_path):
    assert ExportReader(tmp_path).read() is None
    assert ExportReader(tmp_path).wait(timeout_s=0.02) is None
