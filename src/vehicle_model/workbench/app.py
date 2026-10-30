"""HTTP adapter of the workbench (FastAPI): routes only, the logic lives in :mod:`.service` and :mod:`.jobs`.

Requires the optional extra ``workbench`` (``pip install -e .[workbench]``: fastapi, uvicorn).
Bound to 127.0.0.1 by default: a local research tool, no authentication.
"""

from __future__ import annotations

from pathlib import Path

from .jobs import JobManager
from .service import Workbench, WorkbenchError

STATIC = Path(__file__).parent / "static"


def create_app(root: str = "output/workbench", repo: str = "."):
    from fastapi import Body, FastAPI, HTTPException
    from fastapi.responses import FileResponse, RedirectResponse
    from fastapi.staticfiles import StaticFiles

    wb = Workbench(root, repo)
    jobs = JobManager(wb)
    app = FastAPI(title="Vehicle-Dynamics workbench", version="1.0")
    app.state.workbench, app.state.jobs = wb, jobs

    def guard(fn, *a, **k):
        try:
            return fn(*a, **k)
        except WorkbenchError as ex:
            raise HTTPException(status_code=400, detail=str(ex)) from None

    @app.get("/api/options")
    def options():
        return wb.options()

    @app.post("/api/validate")
    def validate(data: dict = Body(...)):
        return wb.validate(data)

    @app.post("/api/configs")
    def save_config(data: dict = Body(...)):
        return guard(wb.save_config, data)

    @app.post("/api/validate_yaml")
    def validate_yaml(body: dict = Body(...)):
        return wb.validate_text(str(body.get("text", "")))

    @app.post("/api/configs_yaml")
    def save_yaml(body: dict = Body(...)):
        return guard(wb.save_text, str(body.get("text", "")))

    @app.get("/api/configs")
    def list_configs():
        return wb.list_configs()

    @app.get("/api/config")
    def read_config(path: str):
        return guard(wb.read_config, path)

    @app.post("/api/jobs")
    def start(body: dict = Body(...)):
        return guard(jobs.start, str(body.get("config", "")))

    @app.get("/api/jobs")
    def list_jobs():
        return jobs.list()

    @app.get("/api/jobs/{jid}")
    def job(jid: str):
        return guard(jobs.status, jid)

    @app.post("/api/jobs/{jid}/cancel")
    def cancel(jid: str):
        return guard(jobs.cancel, jid)

    @app.get("/api/results")
    def results():
        return wb.list_results()

    @app.get("/api/result")
    def result(id: str):
        r = guard(wb.result, id)
        r["reproduce"] = guard(wb.reproduce_command, id)
        return r

    @app.get("/api/history")
    def history(path: str):
        return guard(wb.history, path)

    @app.post("/api/compare")
    def compare(body: dict = Body(...)):
        return guard(wb.compare, list(body.get("ids", [])))

    @app.post("/api/export")
    def export(body: dict = Body(...)):
        return guard(wb.export_report, list(body.get("ids", [])), str(body.get("title", "Workbench comparison")))

    @app.get("/files/{path:path}")
    def files(path: str):
        return FileResponse(guard(wb.file, path))

    @app.get("/")
    def index():
        return RedirectResponse("/ui/index.html")

    app.mount("/ui", StaticFiles(directory=STATIC), name="ui")
    return app


__all__ = ("create_app",)
