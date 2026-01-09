from fastapi import APIRouter, Depends
from sqlmodel import Session, select
from backend.db.session import get_session
from backend.models.MODEL import MODEL, MODELRead, MODELCreate, MODELUpdate
from utils.errors import not_found, bad_request, conflict
from utils.auth import auth_required

router = APIRouter(prefix="/ROUTE_NAME", tags=["ROUTE_NAME"])

# ROUTE_FUNCTIONS
