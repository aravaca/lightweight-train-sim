# Random Mode Notch Preservation Fix

## Problem
When the random-scenario toggle is activated and after the first run completes, the vehicle would reload at notch N (=0) on subsequent runs, instead of maintaining the notch position from the previous run.

## Root Cause
The `reset()` method in `server.py` was unconditionally setting `lever_notch=0` whenever it was called, even in random mode when a previous run had completed with a non-zero notch value.

## Solution

### Server-side Fix (server.py)
Modified the `reset()` method to:
1. Check if random mode is active (`self.random_mode == True`)
2. Check if a previous run has completed and stored the final notch (`self.final_notch_on_finish`)
3. If both conditions are true, preserve the notch value from the previous run
4. Otherwise, start with notch 0 (default behavior for normal mode or first run in random mode)

```python
# ▼ Random mode에서 notch 보존: 이전 실행의 final_notch_on_finish를 사용
if self.random_mode and hasattr(self, 'final_notch_on_finish'):
    prev_lever_notch = int(self.final_notch_on_finish)
else:
    prev_lever_notch = 0

self.state = State(
    t=0.0, s=0.0, v=0.0, a=0.0, lever_notch=prev_lever_notch, finished=False
)
```

### How It Works
1. **First run in random mode**: `final_notch_on_finish` is initialized to 0, so the first run starts at notch 0 (correct)
2. **After first run completes**: `final_notch_on_finish` is set to the notch value when the simulation finished
3. **Subsequent runs with random mode ON**: 
   - User clicks Start (or uses Space to advance via `continueToNextStation`)
   - `setInitial` command is sent with `random_mode=true`
   - Server calls `reset()`
   - Since random_mode=true and final_notch_on_finish has a value from the previous run, the notch is preserved
   - The simulation starts with the preserved notch instead of resetting to 0

## Files Modified
- `/workspaces/emu-personal-type-simulator/tasc/server.py` - Modified `reset()` method

## Testing
The fix should be tested by:
1. Enable random scenario toggle
2. Start a run and stop at a non-zero notch (e.g., B3, B4, etc.)
3. Finish the run
4. Start another run (either by clicking Start or pressing Space)
5. Verify that the notch is preserved (shows the same notch from the previous run instead of resetting to N/0)
