@router.get("/", response_model=List[TaskRead])
async def get_tasks(
    session: Session = Depends(get_session),
    user=Depends(verify_jwt)
):
    tasks = session.exec(
        select(Task).where(Task.user_id == user.id)
    ).all()
    return tasks
