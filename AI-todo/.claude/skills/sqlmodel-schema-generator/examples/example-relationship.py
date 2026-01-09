class Task(SQLModel, table=True):
    user: "User" = Relationship(back_populates="tasks")
