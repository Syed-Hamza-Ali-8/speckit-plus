@router.delete("/{task_id}", status_code=204)
async def delete_task(task_id: str, session: Session = Depends(get_session), user=Depends(verify_jwt)):
    task = session.get(Task, task_id)
    if not task or task.user_id != user.id:
        raise HTTPException(status_code=404)
    session.delete(task)
    session.commit()
