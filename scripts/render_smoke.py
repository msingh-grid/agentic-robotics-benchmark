from pathlib import Path

import cv2
import mujoco
import numpy as np


def main():
    model = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <visual><global offwidth="320" offheight="240"/></visual>
          <worldbody>
            <light pos="0 -1 2"/>
            <geom type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.1 1"/>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.distance = 1.4
    camera.azimuth = 135
    camera.elevation = -25
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render()
    assert rgb.shape == (240, 320, 3)
    assert rgb.dtype == np.uint8
    assert np.count_nonzero(rgb.max(axis=2) - rgb.min(axis=2) > 20) > 100
    output = Path(__file__).resolve().parents[1] / "out" / "smoke.png"
    output.parent.mkdir(exist_ok=True)
    assert cv2.imwrite(str(output), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    restored = cv2.imread(str(output))
    assert restored is not None and restored.shape == rgb.shape
    print(f"Rendered {output}", flush=True)


if __name__ == "__main__":
    main()
