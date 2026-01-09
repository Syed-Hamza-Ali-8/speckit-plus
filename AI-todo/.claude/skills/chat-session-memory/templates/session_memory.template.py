def set_context(session, key, value):
    session.set(key, value)

def get_context(session, key):
    return session.get(key)
