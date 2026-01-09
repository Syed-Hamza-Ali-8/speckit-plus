# API Fetch Guidelines

## Always include:
- method
- headers
- body (if required)

## Include JWT:
Authorization: Bearer <token>

## Use JSON for send + receive:
headers: {
  "Content-Type": "application/json"
}
