import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from sandbox.server import SimServer
from sandbox.sim.render import SceneRenderer
from sandbox.sim.world import CAMERAS, World



def assert_stable_rgb(actual, expected):
    difference = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
    assert difference.max() <= 2
    assert np.mean(difference == 0) > 0.999


def test_reused_renderer_preserves_camera_pixels_and_refreshes_scene():
    world = World(seed=42)
    with SceneRenderer(world.model) as renderer:
        expected = {camera: renderer.render(world.data, camera) for camera in CAMERAS}
        for camera in CAMERAS:
            rgb, depth = renderer.render(world.data, camera)
            assert_stable_rgb(rgb, expected[camera][0])
            np.testing.assert_array_equal(depth, expected[camera][1])
        segmentation = renderer.render(world.data, "cam2", segmentation=True)
        assert segmentation.shape == (480, 640, 2)
        rgb, depth = renderer.render(world.data, "cam2")
        assert_stable_rgb(rgb, expected["cam2"][0])
        np.testing.assert_array_equal(depth, expected["cam2"][1])
        world.set_body_pose("wire0", [0, -0.15, 1.0], [1, 0, 0, 0])
        moved_rgb, moved_depth = renderer.render(world.data, "cam2")
        assert np.any(np.abs(moved_rgb.astype(np.int16) - rgb.astype(np.int16)) > 2)
        assert not np.array_equal(moved_depth, depth)


@pytest.fixture
def camera_server(tmp_path):
    server = SimServer(tmp_path, seed=42)
    try:
        yield server
    finally:
        server.stop.set()
        server.camera_barrier.abort()
        for worker in server.workers:
            if worker.ident is not None:
                worker.join(timeout=10)
            assert not worker.is_alive()
        server.state_socket.close(linger=0)
        server.command_socket.close(linger=0)
        server.context.term()


def test_parallel_cameras_share_fresh_snapshot_and_preserve_pixels(camera_server):
    server = camera_server
    expected = {camera: server.world.render(camera) for camera in CAMERAS}
    for worker in server.workers:
        worker.start()
    frames = [server.frames.get(timeout=20) for _ in CAMERAS]
    assert {frame["name"] for frame in frames} == set(CAMERAS)
    assert len({frame["time_ns"] for frame in frames}) == 1
    baseline = {}
    for frame in frames:
        rgb = np.frombuffer(frame["rgb"], dtype=np.uint8).reshape(480, 640, 3)
        depth = np.frombuffer(frame["depth"], dtype="<f4").reshape(480, 640)
        # Independent CGL contexts can differ at rasterization edges.
        assert np.mean(rgb == expected[frame["name"]][0]) > 0.999
        np.testing.assert_array_equal(depth, expected[frame["name"]][1])
        baseline[frame["name"]] = rgb, depth
    repeated = [server.frames.get(timeout=20) for _ in CAMERAS]
    assert {frame["name"] for frame in repeated} == set(CAMERAS)
    for frame in repeated:
        rgb = np.frombuffer(frame["rgb"], dtype=np.uint8).reshape(480, 640, 3)
        depth = np.frombuffer(frame["depth"], dtype="<f4").reshape(480, 640)
        assert np.mean(rgb == baseline[frame["name"]][0]) > 0.999
        np.testing.assert_array_equal(depth, baseline[frame["name"]][1])
    with server.lock:
        server.world.set_body_pose("wire0", [0, -0.15, 1.0], [1, 0, 0, 0])
        server.step()
    refreshed = [server.frames.get(timeout=20) for _ in CAMERAS]
    assert {frame["name"] for frame in refreshed} == set(CAMERAS)
    assert len({frame["time_ns"] for frame in refreshed}) == 1
    assert refreshed[0]["time_ns"] > frames[0]["time_ns"]
    for frame in refreshed:
        rgb = np.frombuffer(frame["rgb"], dtype=np.uint8).reshape(480, 640, 3)
        depth = np.frombuffer(frame["depth"], dtype="<f4").reshape(480, 640)
        assert np.any(
            np.abs(rgb.astype(np.int16) - baseline[frame["name"]][0].astype(np.int16)) > 2
        )
        assert np.max(np.abs(depth - baseline[frame["name"]][1])) > 0.01


def test_camera_failure_releases_both_render_workers(camera_server, monkeypatch):
    original = SceneRenderer.render

    def render(renderer, data, camera, **kwargs):
        if camera == CAMERAS[0]:
            raise RuntimeError("Camera rendering failed")
        return original(renderer, data, camera, **kwargs)

    monkeypatch.setattr(SceneRenderer, "render", render)
    for worker in camera_server.workers:
        worker.start()
    assert camera_server.stop.wait(timeout=20)
    for worker in camera_server.workers:
        worker.join(timeout=10)
        assert not worker.is_alive()
    assert isinstance(camera_server.render_error, RuntimeError)
    assert str(camera_server.render_error) == "Camera rendering failed"


@pytest.mark.skipif(sys.platform != "linux", reason="Linux headless backends")
def test_egl_failure_falls_back_to_real_osmesa():
    result = subprocess.run(
        [sys.executable, "-c", """
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import mujoco
from sandbox.sim.render import render_scene

context = Mock(side_effect=ImportError("no EGL device"))
sys.modules["mujoco.egl"] = SimpleNamespace(GLContext=context)
model = mujoco.MjModel.from_xml_string('''
<mujoco>
  <worldbody>
    <geom type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <camera name="view" pos="0 0 1"/>
  </worldbody>
</mujoco>
''')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
rgb, depth = render_scene(model, data, "view")
assert np.count_nonzero(rgb[:, :, 0] > rgb[:, :, 1]) > 100
assert abs(depth[240, 320] - 0.8) < 0.002
context.assert_called_once_with(640, 480)
"""],
        cwd=Path(__file__).resolve().parents[1],
        env={key: value for key, value in os.environ.items() if key != "MUJOCO_GL"},
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "using OSMesa" in result.stderr


@pytest.mark.skipif(sys.platform != "linux", reason="Linux software backend")
def test_forced_osmesa_cell_render():
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, str(root / "scripts" / "render_cell.py")],
        cwd=root,
        env={**os.environ, "MUJOCO_GL": "osmesa"},
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
