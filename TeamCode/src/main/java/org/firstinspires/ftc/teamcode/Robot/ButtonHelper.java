package org.firstinspires.ftc.teamcode.Robot;

public class ButtonHelper {
    private boolean lastState = false;

    /**
     * Returns true ONLY on the exact loop the button transitions from unpressed to pressed.
     */
    public boolean wasPressed(boolean currentState) {
        boolean pressed = currentState && !lastState;
        lastState = currentState;
        return pressed;
    }

    /**
     * Returns true ONLY on the exact loop the button transitions from pressed to unpressed.
     */
    public boolean wasReleased(boolean currentState) {
        boolean released = !currentState && lastState;
        lastState = currentState;
        return released;
    }
}