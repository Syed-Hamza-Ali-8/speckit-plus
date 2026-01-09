from enum import Enum
from sqlmodel import Session
from utils.errors import bad_request, not_found, conflict
from backend.services.MODEL_service import (
    get_MODEL,
    update_MODEL
)

class MODELState(str, Enum):
    # STATES
    pass


TRANSITIONS = {
    # MATRIX
}


def validate_transition(from_state: MODELState, to_state: MODELState):
    allowed = TRANSITIONS.get(from_state, [])
    if to_state not in allowed:
        bad_request(f"Invalid transition: {from_state} → {to_state}")


# GUARD_FUNCTIONS

# ACTION_FUNCTIONS

def advance_MODEL(model_id: str, to_state: MODELState, session: Session):
    model = get_MODEL(model_id, session)
    validate_transition(model.state, to_state)

    # Guard checks

    # Entry/Exit actions

    model.state = to_state
    session.add(model)
    session.commit()
    session.refresh(model)
    return model
