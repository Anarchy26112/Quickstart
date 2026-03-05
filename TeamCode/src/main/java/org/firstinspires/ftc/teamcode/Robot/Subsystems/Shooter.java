package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final Telemetry telemetry;

    // Target velocities in ticks per second
    private double targetVelocity = 0.0;
    private double targetRVelocity = 0.0;
    private double targetLVelocity = 0.0;

    private static final double STOP_VELOCITY = 0.0;
    private static final double VELOCITY_THRESHOLD = 50.0;

    // ---- YOUR TUNED PIDF VALUES ----
    // Put these in HamiltonParams if you want.
    private static final double SHOOTER_kP = 50.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 10.5;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        // Directions (mirrored setup)
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        // Flywheels: FLOAT is typical
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset + velocity control mode
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- APPLY CUSTOM PIDF TO BUILT-IN VELOCITY LOOP ----
        applyVelocityPIDF(new PIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        ));
    }

    private void applyVelocityPIDF(PIDFCoefficients pidf) {
        // This sets the internal motor controller coefficients for velocity control
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // Set shooter velocity (ticks/sec) for both motors
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        targetRVelocity = velocity;
        targetLVelocity = velocity;

        rightShooter.setVelocity(velocity);
        leftShooter.setVelocity(velocity);
    }

    // Independent set (if you still want it)
    public void setRVelocity(double velocity) {
        targetRVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        rightShooter.setVelocity(velocity);
    }

    public void setLVelocity(double velocity) {
        targetLVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        leftShooter.setVelocity(velocity);
    }

    // Legacy power -> velocity
    public void setPower(double power) {
        setVelocity(power * SHOOTER_MAX_VELOCITY);
    }

    public void spin(double velocity) {
        setVelocity(Math.abs(velocity));
    }

    public void stop() {
        setVelocity(STOP_VELOCITY);
    }

    public boolean isRunning() {
        return getRightVelocity() > VELOCITY_THRESHOLD ||
                getLeftVelocity() > VELOCITY_THRESHOLD;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetRVelocity() {
        return targetRVelocity;
    }

    public double getTargetLVelocity() {
        return targetLVelocity;
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getAverageVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2.0;
    }

    public double getVelocityError() {
        return targetVelocity - getAverageVelocity();
    }
}