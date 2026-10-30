"""Interactive engineering workbench (Plan V3.0 M15): a web interface over the YAML/CLI experiment engine.

Start: ``python -m vehicle_model.workbench`` (needs the ``workbench`` extra: fastapi, uvicorn), then open
http://127.0.0.1:8765. :mod:`.service` and :mod:`.jobs` hold the logic; :mod:`.app` is the HTTP adapter and
``static/`` the browser front-end.
"""

from .jobs import JobManager
from .service import Workbench, WorkbenchError

__all__ = ("JobManager", "Workbench", "WorkbenchError")
