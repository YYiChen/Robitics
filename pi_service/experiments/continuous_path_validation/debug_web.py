from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

_SOURCE = Path(__file__).resolve().parents[1] / "fixed_rectangle_validation" / "debug_web.py"
_SPEC = spec_from_file_location("rectangle_debug_web", _SOURCE)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError(f"cannot load debug publisher: {_SOURCE}")
_MODULE = module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
DebugMjpegPublisher = _MODULE.DebugMjpegPublisher
