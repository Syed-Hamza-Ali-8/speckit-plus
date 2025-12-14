class User(SQLModel, table=True):
    id: str = Field(primary_key=True)
    email: str
    name: str
    tasks: list["Task"] = Relationship(back_populates="user")
