package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    private final Servo spin_dex;
    private final Telemetry telemetry;

    private int currentPosition = 0;
    private int currentTurn = 0;
    public String[] slots = {"empty", "empty", "empty"};

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        spin_dex = hardwareMap.get(Servo.class, HW_SPINDEX);
    }

    public void moveToPosition(int targetPosition) {
        currentPosition = ((targetPosition % 6) + 6) % 6;
        double servoPos = (720 + currentPosition * 60) / 1620.0;
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        spin_dex.setPosition(servoPos);
    }

    private double calculateServoPosition(int position, int turn) {
        // Each position = 60 degrees
        double degreesPerPosition = 60.0;
        double totalDegrees = (turn * 360.0) + (position * degreesPerPosition);

        // Normalize: 0.0–1.0 corresponds to 0–1800 degrees (5 turns)
        double normalized = totalDegrees / SERVO_TURN_RANGE_DEGREES;

        return normalized;
    }


    // ========== TELEMETRY + STATE ==========

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int getCurrentTurn() {
        return currentTurn;
    }

    public double getServoPosition() {
        return spin_dex != null ? spin_dex.getPosition() : -1.0;
    }

    public int getFilledCount() {
        int count = 0;
        for (String slot : slots) {
            if (!slot.equals("empty")) count++;
        }
        return count;
    }
    public void setToZero(){
        spin_dex.setPosition(Servo.MIN_POSITION);
    }
    public void setToOne(){
        spin_dex.setPosition(Servo.MAX_POSITION);
    }
}