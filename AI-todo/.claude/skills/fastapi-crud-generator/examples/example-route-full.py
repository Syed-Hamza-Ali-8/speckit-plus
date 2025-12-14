from fastapi import APIRouter, Depends, HTTPException
from sqlmodel import Session, select
from models.task import Task, TaskCreate, TaskRead, TaskUpdate
from utils.auth import verify_jwt
from db.session import get_session

router = APIRouter(prefix="/tasks", tags=["Tasks"])

@router.get("/", response_model=list[TaskRead])
async def list_tasks(session: Session = Depends(get_session), user=Depends(verify_jwt)):
    tasks = session.exec(select(Task).where(Task.user_id == user.id)).all()
    return tasks
