package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private static final double VELOCITY_THRESHOLD = 50.0; // Minimum velocity to consider "running"

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        // Set directions (assuming mirrored setup)
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to float for flywheels
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset and configure for velocity control
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set shooter velocity (ticks per second) for both motors
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        targetRVelocity = velocity;
        targetLVelocity = velocity;
        if (rightShooter != null && leftShooter != null) {
            rightShooter.setVelocity(velocity);
            leftShooter.setVelocity(velocity);
        }
    }

    // Set right shooter velocity independently
    public void setRVelocity(double velocity) {
        targetRVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        if (rightShooter != null) {
            rightShooter.setVelocity(velocity);
        }
    }

    // Set left shooter velocity independently
    public void setLVelocity(double velocity) {
        targetLVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        if (leftShooter != null) {
            leftShooter.setVelocity(velocity);
        }
    }

    // ============================================================
    // LEGACY POWER-BASED METHODS (for backwards compatibility)
    // ============================================================

    // Set shooter power (0.0 to 1.0) - converts to velocity
    // Assumes max velocity is around 2800 ticks/sec (adjust for your motor)
    public void setPower(double power) {
        double velocity = power * SHOOTER_MAX_VELOCITY;
        setVelocity(velocity);
    }

    // Set right shooter power independently
    public void setRPower(double power) {
        double velocity = power * SHOOTER_MAX_VELOCITY;
        setRVelocity(velocity);
    }

    // Set left shooter power independently
    public void setLPower(double power) {
        double velocity = power * SHOOTER_MAX_VELOCITY;
        setLVelocity(velocity);
    }

    // Start shooter at specified velocity
    public void spin(double velocity) {
        setVelocity(Math.abs(velocity)); // Ensure positive velocity
    }

    // Stop shooter
    public void stop() {
        setVelocity(STOP_VELOCITY);
    }

    // Check if shooter is running (either motor above threshold)
    public boolean isRunning() {
        return getRightVelocity() > VELOCITY_THRESHOLD ||
                getLeftVelocity() > VELOCITY_THRESHOLD;
    }

    // Check if shooter is stopped (both motors below threshold)
    public boolean isStopped() {
        return getRightVelocity() <= VELOCITY_THRESHOLD &&
                getLeftVelocity() <= VELOCITY_THRESHOLD;
    }

    // ============================================================
    // VELOCITY GETTERS
    // ============================================================

    // Get target velocities
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetRVelocity() {
        return targetRVelocity;
    }

    public double getTargetLVelocity() {
        return targetLVelocity;
    }

    // Get actual motor velocities
    public double getRightVelocity() {
        if (rightShooter != null) {
            return rightShooter.getVelocity();
        }
        return 0.0;
    }

    public double getLeftVelocity() {
        if (leftShooter != null) {
            return leftShooter.getVelocity();
        }
        return 0.0;
    }

    // Get average velocity of both motors
    public double getAverageVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2.0;
    }

    // Get average target velocity
    public double getAverageTargetVelocity() {
        return (targetRVelocity + targetLVelocity) / 2.0;
    }

    // ============================================================
    // POWER GETTERS (for backwards compatibility)
    // ============================================================

    // Get current average "power" (as percentage of max velocity)
    public double getCurrentPower() {
        return targetVelocity / SHOOTER_MAX_VELOCITY;
    }

    // Get individual motor "powers"
    public double getCurrentRPower() {
        return targetRVelocity / SHOOTER_MAX_VELOCITY;
    }

    public double getCurrentLPower() {
        return targetLVelocity / SHOOTER_MAX_VELOCITY;
    }

    // ============================================================
    // UTILITY METHODS
    // ============================================================

    // Get velocity error
    public double getVelocityError() {
        return targetVelocity - getAverageVelocity();
    }
}