# Random Mode Notch Preservation - Test Cases

## Test Case 1: First Run in Random Mode
**Steps:**
1. Toggle random mode ON
2. Click Start button
3. Drive the train and stop at notch B3 (or any non-zero notch)
4. Run finishes, score displayed

**Expected Result:**
- First run should start at notch N (0) ✓
- Notch should properly show B3 when stopped
- After finish, final_notch_on_finish should be set to 3

**Actual Verification:**
- In reset() with random_mode=true and final_notch_on_finish=0 (initial), prev_lever_notch=0 ✓
- State created with lever_notch=0 ✓

---

## Test Case 2: Second Run - Click Start Again (Random Mode)
**Steps:**
1. Complete first run (stop at B3)
2. Click Start button again without using Space
3. Observe the notch at beginning of second run

**Expected Result:**
- Second run should start at notch B3 (preserved from first run) ✓
- Should NOT reset to N (0)

**Actual Verification:**
- In reset() with random_mode=true and final_notch_on_finish=3, prev_lever_notch=3 ✓
- State created with lever_notch=3 ✓
- Debug output: "[RESET] *** RANDOM MODE NOTCH PRESERVATION: Using final_notch_on_finish=3"

---

## Test Case 3: Advance to Next Station (Random Mode)
**Steps:**
1. Complete first run (stop at B3)
2. Press Space key to advance to next station
3. Observe the notch at beginning of second run

**Expected Result:**
- Second run should start at notch B3 (preserved)
- Should NOT reset to N (0)

**Actual Verification:**
- In advanceStation: calculates prev_lever_notch from notch_history or final_notch_on_finish
- Calls sim.reset() which preserves notch via final_notch_on_finish
- Then explicitly sets sim.state.lever_notch = prev_lever_notch again
- Debug output from advanceStation shows: "[ADVANCE] >>> sim.state.lever_notch=3 (should be 3, preserved from previous run)"

---

## Test Case 4: Toggle Random Mode OFF
**Steps:**
1. Complete a run in random mode
2. Toggle random mode OFF
3. Click Start button

**Expected Result:**
- New run should start at notch N (0)
- Should reset to 0 because random_mode=false

**Actual Verification:**
- In reset() with random_mode=false, prev_lever_notch=0 ✓
- State created with lever_notch=0 ✓
- Debug output: "[RESET] Normal reset: lever_notch starting at 0 (random_mode=False)"

---

## Test Case 5: Multiple Runs - Different Notches
**Steps:**
1. Run 1: Stop at B2 (final_notch_on_finish=2)
2. Click Start (should preserve B2)
3. Run 2: Stop at B5 (final_notch_on_finish=5)
4. Click Start (should preserve B5)
5. Run 3: Continue with B5

**Expected Result:**
- Each run preserves the notch from the previous run
- Notch properly updates when a new run finishes at a different notch

**Actual Verification:**
- After Run 1: final_notch_on_finish=2
- Reset for Run 2: prev_lever_notch=2 ✓
- After Run 2: final_notch_on_finish=5
- Reset for Run 3: prev_lever_notch=5 ✓

---

## Code Changes Summary
**File:** `/workspaces/emu-personal-type-simulator/tasc/server.py`

**Method:** `StoppingSim.reset()`

**Change:**
- Added logic to check if random_mode is active and final_notch_on_finish exists
- If both conditions true: preserve the notch from previous run
- Otherwise: reset to 0 (default behavior)

**Key Variables:**
- `self.random_mode`: Set via setInitial command, persists across resets
- `self.final_notch_on_finish`: Set when simulation finishes (line 1365)
- `prev_lever_notch`: Calculated in reset() based on above conditions

---

## Debug Output Expected
When DEBUG=True:
```
[RESET] *** RANDOM MODE NOTCH PRESERVATION: Using final_notch_on_finish=3
[ADVANCE] >>> sim.state.lever_notch=3 (should be 3, preserved from previous run)
```
