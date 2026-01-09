@router.post("/", response_model=TaskRead, status_code=201)
async def create_task(
    payload: TaskCreate,
    session: Session = Depends(get_session),
    user=Depends(verify_jwt)
):
    task = Task(**payload.model_dump(), user_id=user.id)
    session.add(task)
    session.commit()
    session.refresh(task)
    return task
