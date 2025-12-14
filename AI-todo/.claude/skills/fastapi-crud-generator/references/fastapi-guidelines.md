# FastAPI Route Guidelines

## Use APIRouter()
router = APIRouter(prefix="/tasks", tags=["Tasks"])

## Use response_model=
@router.get("/", response_model=List[TaskRead])

## Use Depends
async def get_db():
    with Session(engine) as session:
        yield session

## Use Pydantic for validation
class TaskCreate(BaseModel):
    title: str
