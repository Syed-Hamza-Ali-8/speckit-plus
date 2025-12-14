# JWT Basics

A JWT contains:
header:
  alg: "HS256"
payload:
  id:
  email:
  exp:

Typical Authorization header:
Authorization: Bearer <token>

Backend must decode using secret:
jwt.decode(token, JWT_SECRET, algorithms=["HS256"])
