"""Offscreen rendering without a window server on Linux or macOS."""

from contextlib import ExitStack, contextmanager
import ctypes
from ctypes.util import find_library
import os
import sys
import warnings

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


@contextmanager
def osmesa_context(width, height):
    # Call OSMesa directly: PyOpenGL cannot switch platforms after an EGL failure.
    library = find_library("OSMesa")
    if library is None:
        raise RuntimeError("OSMesa is unavailable; run bash scripts/setup.sh")
    gl = ctypes.CDLL(library)
    gl.OSMesaCreateContextExt.argtypes = (
        ctypes.c_uint, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
    )
    gl.OSMesaCreateContextExt.restype = ctypes.c_void_p
    gl.OSMesaMakeCurrent.argtypes = (
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_int,
    )
    gl.OSMesaMakeCurrent.restype = ctypes.c_ubyte
    gl.OSMesaDestroyContext.argtypes = (ctypes.c_void_p,)
    gl.OSMesaDestroyContext.restype = None
    context = gl.OSMesaCreateContextExt(0x1908, 24, 8, 0, None)  # GL_RGBA
    if not context:
        raise RuntimeError("OSMesa could not create an offscreen context")
    try:
        buffer = np.empty((height, width, 4), dtype=np.uint8)
        if not gl.OSMesaMakeCurrent(context, buffer.ctypes.data, 0x1401, width, height):
            raise RuntimeError("OSMesa could not make its context current")
        yield
    finally:
        gl.OSMesaDestroyContext(context)


@contextmanager
def offscreen_context(width, height):
    if sys.platform == "darwin":
        with cgl_context():
            yield
        return
    if sys.platform != "linux":
        raise RuntimeError(f"Unsupported rendering platform: {sys.platform}")
    if os.environ.get("MUJOCO_GL") != "osmesa":
        resources = ExitStack()
        try:
            from mujoco.egl import GLContext

            context = GLContext(width, height)
            resources.callback(context.free)
            context.make_current()
        except Exception as error:
            resources.close()
            warnings.warn(f"EGL unavailable; using OSMesa: {error}", RuntimeWarning)
        else:
            with resources:
                yield
            return
    with osmesa_context(width, height):
        yield


class SceneRenderer:
    """Thread-bound offscreen renderer that retains uploaded model resources."""

    def __init__(self, model, *, width=640, height=480):
        self.model = model
        self.width = width
        self.height = height
        self.resources = ExitStack()

    def __enter__(self):
        try:
            self.resources.enter_context(offscreen_context(self.width, self.height))
            self.scene = mujoco.MjvScene(self.model, maxgeom=max(1000, self.model.ngeom * 2))
            self.option = mujoco.MjvOption()
            self.option.geomgroup[3] = 0  # Collision proxies obscure the robot's visual meshes.
            self.framebuffer = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.resources.callback(self.framebuffer.free)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.framebuffer)
            self.framebuffer.readDepthMap = mujoco.mjtDepthMap.mjDEPTH_ZERONEAR
            self.viewport = mujoco.MjrRect(0, 0, self.width, self.height)
            return self
        except Exception:
            self.resources.close()
            raise

    def __exit__(self, *exc_info):
        return self.resources.__exit__(*exc_info)

    def render(self, data, camera, *, segmentation=False):
        model, scene, framebuffer = self.model, self.scene, self.framebuffer
        if isinstance(camera, str):
            view = mujoco.MjvCamera()
            view.type = mujoco.mjtCamera.mjCAMERA_FIXED
            view.fixedcamid = model.camera(camera).id
        else:
            view = camera
        mujoco.mjv_updateScene(
            model, data, self.option, None, view, mujoco.mjtCatBit.mjCAT_ALL, scene,
        )
        rgb = np.empty((self.height, self.width, 3), dtype=np.uint8)
        depth = np.empty((self.height, self.width), dtype=np.float32)
        scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = segmentation
        scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = segmentation
        mujoco.mjr_render(self.viewport, scene, framebuffer)
        mujoco.mjr_readPixels(rgb, depth, self.viewport, framebuffer)
        if segmentation:
            colors = rgb.astype(np.uint32)
            ids = colors[..., 0] + 256 * colors[..., 1] + 65536 * colors[..., 2]
            objects = np.full((scene.ngeom + 1, 2), -1, dtype=np.int32)
            for geom in scene.geoms[:scene.ngeom]:
                if geom.segid >= 0:
                    objects[geom.segid + 1] = geom.objid, geom.objtype
            return np.flipud(objects[ids]).copy()
        near = model.vis.map.znear * model.stat.extent
        far = model.vis.map.zfar * model.stat.extent
        depth = (near / (1 - depth.astype(np.float64) * (1 - near / far))).astype(np.float32)
        return np.flipud(rgb).copy(), np.flipud(depth).copy()


def render_scene(model, data, camera, *, width=640, height=480, segmentation=False):
    """Return top-left-origin RGB/depth, or (object id, object type) segmentation."""
    with SceneRenderer(model, width=width, height=height) as renderer:
        return renderer.render(data, camera, segmentation=segmentation)
