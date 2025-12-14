---
id: 004
title: Auth UI Clarification
stage: spec
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-auth-ui
branch: main
user: user
command: /sp.clarify Phase 3 Part 2: Authentication UI
labels: ["clarification", "spec-refinement", "ux-decisions", "authentication"]
links:
  spec: specs/phase3/part2-auth-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase3/part2-auth-ui/spec.md
tests:
  - null
---

## Prompt

Run /sp.clarify for Phase 3 Part 2: Authentication UI with 7 clarification questions:

1. Form Validation Errors Display: Inline field errors vs summary vs both?
2. Password Visibility Toggle: Show/hide password icon in input field?
3. Toast Auto-dismiss Timing: 3s success/5s error vs 5s all vs manual dismiss?
4. Login Success Flow: Immediate redirect vs show success toast first?
5. Register Success Behavior: Auto-login after register vs redirect to login page?
6. Network Error Messages: Generic message vs detailed technical errors?
7. Mobile Keyboard Behavior: Auto-scroll form into view or no special handling?

User confirmed:
- Q1: A - Inline field errors (below each invalid field)
- Q2: A - Yes, add eye icon toggle to show/hide password
- Q3: A - 3 seconds for success, 5 seconds for errors
- Q4: A - Immediate redirect after login
- Q5: A - Auto-login after registration, redirect to /tasks
- Q6: A - Generic "Connection error. Please try again."
- Q7: A - Auto-scroll form into view when keyboard appears

## Response snapshot

Updated `specs/phase3/part2-auth-ui/spec.md` with clarification answers:

**Added/Updated Sections:**

1. **Section 1.2 Toast Notifications** - Added auto-dismiss timing column:
   - Success toasts: 3 seconds
   - Error toasts: 5 seconds

2. **Section 1.5 Redirect Behavior** - Updated:
   - Login success: Immediate redirect (no delay)
   - Register success: Auto-login after registration

3. **Section 1.6 Password Field UX** - NEW:
   - Eye icon toggle for password visibility
   - Default state: password masked
   - Toggle behavior documented

4. **Section 1.7 Mobile UX** - NEW:
   - Auto-scroll form into view when keyboard appears
   - Smooth scroll to focused field

5. **Section 9 Clarification Answers** - NEW:
   - Complete table of all 7 Q&A pairs
   - Date confirmed: 2025-12-15

## Outcome

- Impact: Spec refined with UX details ready for implementation
- Tests: N/A (spec phase)
- Files: 1 file modified (spec.md)
- Next prompts: /sp.implement to execute tasks
- Reflection: Clarification questions resolved edge cases before implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
