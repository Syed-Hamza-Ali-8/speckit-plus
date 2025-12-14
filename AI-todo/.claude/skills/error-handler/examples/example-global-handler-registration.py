from utils.exception_handlers import add_exception_handlers

def create_app():
    app = FastAPI()
    add_exception_handlers(app)
    return app
