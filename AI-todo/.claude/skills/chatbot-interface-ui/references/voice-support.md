# Voice Support (Optional)

## Description

Chatkit may be used with voice input/output
by integrating backend STT and TTS pipelines.

---

## Rules

- Voice input is converted to text before agent execution
- Agent produces text output
- Text output may be converted to audio via TTS
- Voice processing must occur server-side

---

## Constraints

- UI only streams audio
- Voice logic must not bypass guardrails
