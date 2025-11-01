package org.firstinspires.ftc.teamcode.Robot;

public class ButtonHelper {
    private boolean lastState = false;

    /**
     * Checks if the button transitioned from UP (false) to DOWN (true) in the current cycle.
     * Updates the internal state for the next check.
     *
     * @param currentState The current state of the button (true if pressed, false otherwise).
     * @return true if the button was just pressed this cycle, false otherwise.
     */
    public boolean wasPressed(boolean currentState) {
        boolean pressed = currentState && !lastState;
        lastState = currentState;
        return pressed;
    }

    /**
     * Resets the helper, clearing the last state.
     * Useful after an Emergency Stop.
     */
    public void reset() {
        lastState = false;
    }

    public boolean getLastState() {
        return lastState;
    }
}
