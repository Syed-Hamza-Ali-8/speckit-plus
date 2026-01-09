# SQLModel Basics

class Example(SQLModel, table=True):
    id: str = Field(default_factory=uuid4, primary_key=True)
    name: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

Model types:
- Base (SQLModel)
- Create (Pydantic input)
- Update (partial update)
- Read (output model)
