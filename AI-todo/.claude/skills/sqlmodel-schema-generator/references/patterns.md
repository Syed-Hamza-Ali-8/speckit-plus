# SQLModel Relationship Patterns

## One-to-Many Example:
class User(SQLModel, table=True):
    id: str = Field(primary_key=True)
    tasks: List["Task"] = Relationship(back_populates="user")

class Task(SQLModel, table=True):
    id: str = Field(primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    user: Optional[User] = Relationship(back_populates="tasks")
