def advance_task(id, to_state, session):
    task = get_task(id, session)
    validate_transition(task.state, to_state)
    task.state = to_state
    session.commit()
    return task
