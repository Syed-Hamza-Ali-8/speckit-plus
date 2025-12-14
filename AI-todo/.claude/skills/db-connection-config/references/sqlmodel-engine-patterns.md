# SQLModel Engine Patterns

## Sync Engine
engine = create_engine(DATABASE_URL, echo=False)

## Async Engine
engine = create_async_engine(DATABASE_URL, echo=False)

## Session Pattern
def get_session():
    with Session(engine) as session:
        yield session
