@router.put("/{task_id}", response_model=TaskRead)
async def update_task(
    task_id: str,
    payload: TaskUpdate,
    session: Session = Depends(get_session),
    user=Depends(verify_jwt)
):
    task = session.get(Task, task_id)
    if not task or task.user_id != user.id:
        raise HTTPException(status_code=404)
    for key, value in payload.model_dump(exclude_unset=True).items():
        setattr(task, key, value)
    session.add(task)
    session.commit()
    session.refresh(task)
    return task
