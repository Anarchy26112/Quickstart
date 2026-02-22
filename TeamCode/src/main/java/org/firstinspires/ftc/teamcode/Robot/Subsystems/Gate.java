package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Gate {

    private final Servo gateServo;
    private final Telemetry telemetry;

    // Simple state tracking (optional but helpful)
    private boolean isBlocking = true;

    public Gate(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gateServo = hardwareMap.get(Servo.class, HW_GATE);

        // Start in blocking position
        block();
    }

    // Set gate to blocking position
    public void block() {
        if (gateServo != null) {
            gateServo.setPosition(GATE_BLOCK_POS);
            isBlocking = true;
        }
    }

    // Set gate to open (not blocking)
    public void open() {
        if (gateServo != null) {
            gateServo.setPosition(GATE_OPEN_POS);
            isBlocking = false;
        }
    }

    // Toggle between open and block
    public void toggle() {
        if (isBlocking) {
            open();
        } else {
            block();
        }
    }

    // Returns current state
    public boolean isBlocking() {
        return isBlocking;
    }

    // Get servo position (0 - 1)
    public double getServoPosition() {
        return (gateServo != null) ? gateServo.getPosition() : -1.0;
    }
}