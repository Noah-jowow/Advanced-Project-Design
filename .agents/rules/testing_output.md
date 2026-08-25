---
description: Rule to enforce showing terminal test output
always_on: true
---

# Testing Output Verification

When running automated tests or validations (e.g. `pytest`, `npm test`, etc.) to verify that bugs have been resolved, **always** show the console output of the final passing test run to the user. This proves that the bug has actually been resolved before moving on to the next task. Never simply state that "the tests passed" without providing the concrete terminal trace.
