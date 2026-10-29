import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from sandbox.sim.render import SceneRenderer
from sandbox.sim.world import CAMERAS, World



def test_reused_renderer_preserves_camera_pixels_and_refreshes_scene():
    world = World(seed=42)
    expected = {camera: world.render(camera) for camera in CAMERAS}
    with SceneRenderer(world.model) as renderer:
        for camera in CAMERAS:
            actual = renderer.render(world.data, camera)
            for image, baseline in zip(actual, expected[camera]):
                np.testing.assert_array_equal(image, baseline)
        segmentation = renderer.render(world.data, "cam2", segmentation=True)
        assert segmentation.shape == (480, 640, 2)
        rgb, depth = renderer.render(world.data, "cam2")
        np.testing.assert_array_equal(rgb, expected["cam2"][0])
        np.testing.assert_array_equal(depth, expected["cam2"][1])
        world.set_body_pose("wire0", [0, -0.15, 1.0], [1, 0, 0, 0])
        moved_rgb, moved_depth = renderer.render(world.data, "cam2")
        assert not np.array_equal(moved_rgb, rgb)
        assert not np.array_equal(moved_depth, depth)


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
