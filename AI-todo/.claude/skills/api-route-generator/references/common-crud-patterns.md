result = session.exec(select(Model)…).first()
if not result:
    not_found("Item not found")
