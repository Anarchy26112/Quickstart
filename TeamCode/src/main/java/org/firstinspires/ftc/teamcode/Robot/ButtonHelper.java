package org.firstinspires.ftc.teamcode.Robot;

public class ButtonHelper {
    private boolean lastState = false;
    public boolean wasPressed(boolean currentState) {
        boolean pressed = currentState && !lastState;
        lastState = currentState;
        return pressed;
    }

    // Resets the helper, clearing the last state.

    public void reset() {
        lastState = false;
    }

    public boolean getLastState() {
        return lastState;
    }
}
