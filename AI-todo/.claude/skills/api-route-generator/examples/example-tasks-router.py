router = APIRouter(prefix="/tasks", tags=["tasks"])

@router.get("/", response_model=list[TaskRead])
def list_tasks(session: Session = Depends(get_session)):
    return session.exec(select(Task)).all()
