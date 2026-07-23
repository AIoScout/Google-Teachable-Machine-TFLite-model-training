# Debug Session: pyinstaller-tensorflow-crash
- **Status**: [OPEN]
- **Issue**: PyInstaller crashes while collecting TensorFlow submodules from `TFLiteTraining.spec`, ending with `SubprocessDiedError` and `libc++abi ... std::__1::system_error: mutex lock failed: Invalid argument`.
- **Debug Server**: Pending
- **Log File**: .dbg/trae-debug-log-pyinstaller-tensorflow-crash.ndjson

## Reproduction Steps
1. Run the PyInstaller build that uses `AItraining/TFLiteTraining.spec`.
2. Observe the build fail during `collect_all('tensorflow')`.
3. Observe the isolated child process abort with exit code `-6`.

## Hypotheses & Verification
| ID | Hypothesis | Likelihood | Effort | Evidence |
|----|------------|------------|--------|----------|
| A | The crash is caused by `collect_all('tensorflow')` importing TensorFlow submodules under Python 3.13, which triggers an unsupported native runtime path in the isolated child process. | High | Low | Pending |
| B | The crash is caused by the current PyInstaller + TensorFlow version combination, not by the app code, and can be reproduced with a minimal isolated hook call. | High | Low | Pending |
| C | The spec file is collecting far more TensorFlow modules than needed, and the child process dies while importing optional native backends. | Med | Med | Pending |
| D | The active interpreter / packaged environment is mixing incompatible native libraries, causing the child process to abort during module discovery. | Med | Med | Pending |
| E | A narrower hidden-import strategy can avoid the crashing collection path without affecting the app build. | Med | Med | Pending |

## Log Evidence
Pending.

## Verification Conclusion
Pending.
