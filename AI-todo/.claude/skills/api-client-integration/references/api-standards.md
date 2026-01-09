# API Standards

## Methods:
- GET for retrieval
- POST for creation
- PUT for updates
- DELETE for removal
- PATCH for partial updates

## Return Format:
Always return parsed JSON:
const data = await res.json();

## Error Rules:
If !res.ok, throw new Error.
