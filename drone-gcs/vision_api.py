"""Human / IR routes hosted by the launcher, independent of serial modules."""
import sys
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse, Response
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from vision_service import VisionService

router = APIRouter()
service = VisionService()


class StartRequest(BaseModel):
    mode: str = 'simulation'
    message: str
    unit: float
    camera_config: str | None = None


@router.get('/vision', response_class=HTMLResponse)
def page():
    return Path(__file__).with_name('vision.html').read_text(encoding='utf-8')


@router.get('/api/vision/status')
def status():
    return service.snapshot()


@router.post('/api/vision/start')
def start(request: StartRequest):
    try:
        service.start(**request.model_dump())
    except (ValueError, OSError) as exc:
        raise HTTPException(400, str(exc)) from exc
    return service.snapshot()


@router.post('/api/vision/stop')
def stop():
    service.stop()
    return service.snapshot()


@router.get('/api/vision/frame')
def frame():
    with service.lock:
        jpeg = service.jpeg if service.snapshot()['fresh'] else None
    if jpeg is None:
        return Response(status_code=204, headers={'Cache-Control': 'no-store'})
    return Response(jpeg, media_type='image/jpeg', headers={'Cache-Control': 'no-store'})
