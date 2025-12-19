# MCP Server Rules & Constraints

## Official SDK

MCP Server must be implemented using the
Official Model Context Protocol SDK.

---

## Rules

- MCP tools MUST be stateless
- MCP tools MUST have strict schemas
- MCP tools MUST return structured data
- MCP server MUST NOT contain AI logic
- MCP server MUST NOT manage UI or sessions

---

## Security Constraints

- MCP tools must receive authenticated user context
- MCP server must not expose database internals
- Guardrails must be enforced before tool execution

---

## Role in Architecture

MCP server acts as the boundary between:
- AI agents
- backend task operations
