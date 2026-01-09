class GuardrailViolation(Exception):
    pass

def require_confirmation(action: str, confirmed: bool):
    if action in ["delete", "update"] and not confirmed:
        raise GuardrailViolation("CONFIRMATION_REQUIRED")

def validate_result(result):
    if not hasattr(result, "final_output"):
        raise GuardrailViolation("INVALID_RESULT")
