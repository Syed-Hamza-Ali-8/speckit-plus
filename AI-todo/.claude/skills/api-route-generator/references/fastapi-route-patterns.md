# FastAPI Route Patterns

router = APIRouter(prefix="/resource", tags=["resource"])

@router.get("/", response_model=list[ReadModel])
def list_items(session: Session = Depends(get_session)):
    ...

@router.post("/", response_model=ReadModel)
def create_item(data: CreateModel, session: Session = Depends(get_session)):
    ...
