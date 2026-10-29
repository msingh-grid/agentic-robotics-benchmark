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


def render_scene(model, data, camera, *, width=640, height=480, segmentation=False):
    """Return top-left-origin RGB/depth, or (object id, object type) segmentation."""
    if isinstance(camera, str):
        view = mujoco.MjvCamera()
        view.type = mujoco.mjtCamera.mjCAMERA_FIXED
        view.fixedcamid = model.camera(camera).id
    else:
        view = camera
    with ExitStack() as resources:
        resources.enter_context(offscreen_context(width, height))
        scene = mujoco.MjvScene(model, maxgeom=max(1000, model.ngeom * 2))
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0  # Collision proxies obscure the robot's visual meshes.
        mujoco.mjv_updateScene(
            model, data, option, None, view, mujoco.mjtCatBit.mjCAT_ALL, scene,
        )
        framebuffer = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        resources.callback(framebuffer.free)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, framebuffer)
        framebuffer.readDepthMap = mujoco.mjtDepthMap.mjDEPTH_ZERONEAR
        viewport = mujoco.MjrRect(0, 0, width, height)
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        depth = np.empty((height, width), dtype=np.float32)
        if segmentation:
            scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = 1
            scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = 1
        mujoco.mjr_render(viewport, scene, framebuffer)
        mujoco.mjr_readPixels(rgb, depth, viewport, framebuffer)
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
