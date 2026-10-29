from contextlib import ExitStack, contextmanager
import ctypes
from pathlib import Path
import sys

import cv2
import mujoco
import numpy as np


@contextmanager
def cgl_context():
    from mujoco.cgl import cgl

    attribute = cgl.CGLPixelFormatAttribute
    buffers = (
        attribute.CGLPFAColorSize, 24,
        attribute.CGLPFAAlphaSize, 8,
        attribute.CGLPFADepthSize, 24,
        attribute.CGLPFAStencilSize, 8,
    )
    # GPU-less Macs expose Apple's GenericFloat software renderer.
    renderers = ((attribute.CGLPFAAccelerated,), (attribute.CGLPFARendererID, 0x00020400))
    with ExitStack() as resources:
        for renderer in renderers:
            pixel = cgl.CGLPixelFormatObj()
            count = cgl.GLint()
            values = (*buffers, *renderer, 0)
            try:
                cgl.CGLChoosePixelFormat(
                    (ctypes.c_int * len(values))(*values),
                    ctypes.byref(pixel), ctypes.byref(count),
                )
            except cgl.CGLError:
                if pixel:
                    cgl.CGLReleasePixelFormat(pixel)
                if renderer == renderers[-1]:
                    raise
                continue
            if pixel and count.value:
                break
            if pixel:
                cgl.CGLReleasePixelFormat(pixel)
        else:
            raise RuntimeError("CGL returned no usable pixel format")
        resources.callback(cgl.CGLReleasePixelFormat, pixel)
        context = cgl.CGLContextObj()
        cgl.CGLCreateContext(pixel, 0, ctypes.byref(context))
        resources.callback(cgl.CGLReleaseContext, context)
        cgl.CGLSetCurrentContext(context)
        resources.callback(cgl.CGLSetCurrentContext, None)
        yield


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
    with ExitStack() as resources:
        if sys.platform == "darwin":
            resources.enter_context(cgl_context())
        else:
            context = mujoco.GLContext(320, 240)
            resources.callback(context.free)
            context.make_current()
        scene = mujoco.MjvScene(model, maxgeom=100)
        mujoco.mjv_updateScene(
            model, data, mujoco.MjvOption(), None, camera,
            mujoco.mjtCatBit.mjCAT_ALL, scene,
        )
        framebuffer = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        resources.callback(framebuffer.free)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, framebuffer)
        viewport = mujoco.MjrRect(0, 0, 320, 240)
        mujoco.mjr_render(viewport, scene, framebuffer)
        rgb = np.empty((240, 320, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, viewport, framebuffer)
        rgb = np.flipud(rgb)
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
