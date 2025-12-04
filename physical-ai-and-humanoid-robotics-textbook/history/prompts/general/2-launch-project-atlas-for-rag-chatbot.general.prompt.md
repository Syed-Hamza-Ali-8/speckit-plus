---
id: 2
title: Launch Project Atlas for RAG Chatbot
stage: general
date: 2025-11-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-foundation-setup
user: user
command: "default_api.Task(description = \"Manage RAG Chatbot Development\", prompt = \"Oversee the development of an integrated RAG chatbot for the Physical AI & Humanoid Robotics textbook. This includes leveraging OpenAI Agents/ChatKit SDKs, building a FastAPI backend, integrating with Neon Serverless Postgres for data storage, and using Qdrant Cloud Free Tier for vector search. The chatbot must answer user questions about the book's content, including text selected by the user. Break down this project into actionable steps and delegate to appropriate subagents.\", subagent_type = \"project-atlas\")"
labels: ["RAG", "chatbot", "FastAPI", "Neon Postgres", "Qdrant", "OpenAI SDK", "project-atlas", "tool-use"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/general/2-launch-project-atlas-for-rag-chatbot.general.prompt.md
tests:
  - "None"
---

## Prompt

I will launch the `project-atlas` agent to manage the development of the RAG chatbot. This agent is well-suited for high-level project management, breaking down complex requirements, and delegating tasks to specialized subagents.

## Response snapshot

I have launched the `project-atlas` agent to manage the development of the RAG chatbot.

## Outcome

- ✅ Impact: `project-atlas` agent launched successfully to oversee RAG chatbot development.
- 🧪 Tests: No tests run.
- 📁 Files: history/prompts/general/2-launch-project-atlas-for-rag-chatbot.general.prompt.md created.
- 🔁 Next prompts: None, `project-atlas` agent will handle further steps.
- 🧠 Reflection: Successfully recorded the action of launching the `project-atlas` agent in a PHR.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
