from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sandbox.sim.world import CAMERAS, World


def main():
    world = World(seed=0)
    views = []
    for camera in CAMERAS:
        rgb, _ = world.render(camera)
        view = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.putText(view, camera, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        views.append(view)
    sheet = np.vstack((np.hstack(views[:2]), np.hstack(views[2:])))
    output = Path(__file__).resolve().parents[1] / "out" / "cell.png"
    output.parent.mkdir(exist_ok=True)
    if not cv2.imwrite(str(output), sheet):
        raise RuntimeError(f"Could not write {output}")
    print(f"Rendered {output}", flush=True)


if __name__ == "__main__":
    main()
