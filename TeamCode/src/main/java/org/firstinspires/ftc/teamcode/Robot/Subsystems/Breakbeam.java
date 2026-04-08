package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Breakbeam {

    private final DigitalChannel sensor;

    // Tracks previous beam state
    private boolean previousBroken = false;

    // Ball count (based on number of breaks)
    private int count = 0;

    private static final int MAX_COUNT = 3;

    public Breakbeam(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(DigitalChannel.class, name);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Most FTC breakbeam sensors:
     * true  = beam NOT broken
     * false = beam broken
     * Flip if yours is reversed.
     */
    public boolean isBroken() {
        return !sensor.getState();
    }

    /**
     * Call every loop()
     * Counts ONE ball per distinct beam break
     */
    public void update() {
        boolean currentBroken = isBroken();

        // Detect NEW break (edge trigger)
        if (currentBroken && !previousBroken) {
            if (count < MAX_COUNT) {
                count++;
            }
        }

        previousBroken = currentBroken;
    }

    public int getCount() {
        return count;
    }

    public boolean isFull() {
        return count >= MAX_COUNT;
    }

    public boolean isEmpty() {
        return count == 0;
    }

    public boolean canIntake() {
        return count < MAX_COUNT;
    }

    /**
     * Call when you shoot or remove a ball
     */
    public void removeOne() {
        if (count > 0) {
            count--;
        }
    }

    /**
     * Reset (start of match or mismatch)
     */
    public void reset() {
        count = 0;
    }
}