from backend.db.session import get_session

session: Session = Depends(get_session)
