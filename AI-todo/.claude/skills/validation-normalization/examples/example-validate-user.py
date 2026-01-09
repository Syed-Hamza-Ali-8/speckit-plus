from templates.template-validation import validate_payload, UserPayload

data = {"username": " John ", "email": "JOHN@EXAMPLE.COM"}
validated, errors = validate_payload(UserPayload, data)
if errors:
    print("Validation errors:", errors)
else:
    print("Validated payload:", validated.dict())
