def query_logs(session, actor_id=None, action_type=None, resource_id=None, resource_type=None, start=None, end=None):
    query = session.query(ActivityLog)
    if actor_id:
        query = query.filter(ActivityLog.actor_id == actor_id)
    if action_type:
        query = query.filter(ActivityLog.action_type == action_type)
    if resource_id:
        query = query.filter(ActivityLog.resource_id == resource_id)
    if resource_type:
        query = query.filter(ActivityLog.resource_type == resource_type)
    if start:
        query = query.filter(ActivityLog.timestamp >= start)
    if end:
        query = query.filter(ActivityLog.timestamp <= end)
    return query.all()
