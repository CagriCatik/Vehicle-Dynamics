"""``python -m vehicle_model.workbench [--host 127.0.0.1] [--port 8765]`` (run from the repository root)."""

from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m vehicle_model.workbench", description="Vehicle-Dynamics workbench")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--root", default="output/workbench", help="workbench directory (configs, runs, exports)")
    a = ap.parse_args(argv)
    try:
        import uvicorn
    except ImportError:
        print("the workbench needs the optional extra: pip install -e '.[workbench]'", file=sys.stderr)
        return 1
    from .app import create_app
    print(f"workbench on http://{a.host}:{a.port}  (Ctrl+C to stop)")
    uvicorn.run(create_app(a.root), host=a.host, port=a.port, log_level="warning")
    return 0


if __name__ == "__main__":
    sys.exit(main())
