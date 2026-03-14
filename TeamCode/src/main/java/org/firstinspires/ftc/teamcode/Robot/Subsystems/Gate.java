package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Gate {

    private final Servo gateServo;
    private final Telemetry telemetry;

    private boolean isBlocking = true;

    // Last hardware-written position (write caching)
    private double lastWrittenPosition = -1.0;
    private static final double WRITE_TOLERANCE = 0.001;

    public Gate(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gateServo = hardwareMap.get(Servo.class, HW_GATE);

        // Start in blocking position
        block();
    }

    public void block() {
        if (gateServo != null) {
            isBlocking = true;

            if (Math.abs(lastWrittenPosition - GATE_BLOCK_POS) > WRITE_TOLERANCE) {
                gateServo.setPosition(GATE_BLOCK_POS);
                lastWrittenPosition = GATE_BLOCK_POS;
            }
        }
    }

    public void open() {
        if (gateServo != null) {
            isBlocking = false;

            if (Math.abs(lastWrittenPosition - GATE_OPEN_POS) > WRITE_TOLERANCE) {
                gateServo.setPosition(GATE_OPEN_POS);
                lastWrittenPosition = GATE_OPEN_POS;
            }
        }
    }

    public void toggle() {
        if (isBlocking) {
            open();
        } else {
            block();
        }
    }

    public boolean isBlocking() {
        return isBlocking;
    }

    public double getServoPosition() {
        return lastWrittenPosition;
    }
}