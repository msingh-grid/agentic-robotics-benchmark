import ctypes
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.mark.parametrize("accelerated", [True, False])
def test_cgl_context_selection_and_cleanup(monkeypatch, accelerated):
    spec = spec_from_file_location(
        "render_smoke", Path(__file__).resolve().parents[1] / "scripts" / "render_smoke.py"
    )
    render_smoke = module_from_spec(spec)
    spec.loader.exec_module(render_smoke)
    requests = []

    def choose_pixel_format(attributes, pixel, count):
        requests.append(list(attributes))
        if not accelerated and len(requests) == 1:
            raise RuntimeError("invalid pixel format")
        ctypes.cast(pixel, ctypes.POINTER(ctypes.c_void_p))[0] = 1
        ctypes.cast(count, ctypes.POINTER(ctypes.c_int))[0] = 1

    def create_context(pixel, shared, context):
        ctypes.cast(context, ctypes.POINTER(ctypes.c_void_p))[0] = 2

    cgl = SimpleNamespace(
        CGLError=RuntimeError,
        CGLPixelFormatObj=ctypes.c_void_p,
        CGLContextObj=ctypes.c_void_p,
        GLint=ctypes.c_int,
        CGLPixelFormatAttribute=SimpleNamespace(
            CGLPFAColorSize=8, CGLPFAAlphaSize=11, CGLPFADepthSize=12,
            CGLPFAStencilSize=13, CGLPFAAccelerated=73, CGLPFARendererID=70,
        ),
        CGLChoosePixelFormat=choose_pixel_format,
        CGLCreateContext=create_context,
        CGLSetCurrentContext=Mock(),
        CGLReleaseContext=Mock(),
        CGLReleasePixelFormat=Mock(),
    )
    monkeypatch.setitem(sys.modules, "mujoco.cgl", SimpleNamespace(cgl=cgl))
    with pytest.raises(ValueError, match="render failed"):
        with render_smoke.cgl_context():
            assert cgl.CGLSetCurrentContext.call_args.args[0].value == 2
            raise ValueError("render failed")
    assert len(requests) == (1 if accelerated else 2)
    assert 73 in requests[0]
    if not accelerated:
        assert requests[1][-3:] == [70, 0x00020400, 0]
        assert 73 not in requests[1]
    cgl.CGLSetCurrentContext.assert_called_with(None)
    assert cgl.CGLReleaseContext.call_args.args[0].value == 2
    assert cgl.CGLReleasePixelFormat.call_args.args[0].value == 1
