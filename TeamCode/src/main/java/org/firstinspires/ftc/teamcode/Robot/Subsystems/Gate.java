package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Gate {

    private final Servo gateServo;
    private boolean isBlocking = true;
    private double lastWrittenPosition = -1.0;

    public Gate(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, HW_GATE);
        block();
    }

    public void block() {
        if (lastWrittenPosition != GATE_BLOCK_POS) {
            gateServo.setPosition(GATE_BLOCK_POS);
            lastWrittenPosition = GATE_BLOCK_POS;
        }
        isBlocking = true;
    }

    public void open() {
        if (lastWrittenPosition != GATE_OPEN_POS) {
            gateServo.setPosition(GATE_OPEN_POS);
            lastWrittenPosition = GATE_OPEN_POS;
        }
        isBlocking = false;
    }

    public void toggle() {
        if (isBlocking) open();
        else            block();
    }

    public boolean isBlocking() {
        return isBlocking;
    }

    public double getServoPosition() {
        return lastWrittenPosition;
    }
}