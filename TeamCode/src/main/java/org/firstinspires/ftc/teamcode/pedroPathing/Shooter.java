package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;
    private TelemetryManager telemetryM;
    private double currentPower = 0.0;

    // Motor configuration
    private static final String RIGHT_SHOOTER_NAME = "rightShooter";
    private static final String LEFT_SHOOTER_NAME = "leftShooter";

    // Power constants
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01; // Minimum power to consider "running"

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetryM = telemetry;


        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER_NAME);
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER_NAME);

        // Set directions (assuming mirrored setup)
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to float for flywheels
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders for velocity monitoring
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Set shooter power (0.0 to 1.0)
    public void setPower(double power) {
        currentPower = power;
        if (rightShooter != null && leftShooter != null) {
            rightShooter.setPower(power);
            leftShooter.setPower(power);
        }
    }

    // Start shooter at specified power
    public void spin(double power) {
        setPower(Math.abs(power)); // Ensure positive power
    }

    // Stop shooter
    public void stop() {
        setPower(STOP_POWER);
    }

    // Check if shooter is running
    public boolean isRunning() {
        return currentPower > POWER_THRESHOLD;
    }

    // Check if shooter is stopped
    public boolean isStopped() {
        return currentPower <= POWER_THRESHOLD;
    }

    // Get current power setting
    public double getCurrentPower() {
        return currentPower;
    }

    // Get actual motor velocities (useful for monitoring)
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

    // Check if both motors are spinning at similar speeds (within tolerance)
    public boolean isBalanced(double tolerancePercent) {
        double rightVel = Math.abs(getRightVelocity());
        double leftVel = Math.abs(getLeftVelocity());

        if (rightVel < POWER_THRESHOLD && leftVel < POWER_THRESHOLD) {
            return true; // Both stopped = balanced
        }

        double avgVel = (rightVel + leftVel) / 2.0;
        double difference = Math.abs(rightVel - leftVel);
        double percentDifference = (difference / avgVel) * 100.0;

        return percentDifference <= tolerancePercent;
    }

    // Check if shooter is at target speed (within tolerance)
    public boolean isAtSpeed(double targetVelocity, double tolerancePercent) {
        double avgVel = getAverageVelocity();
        double difference = Math.abs(avgVel - targetVelocity);
        double percentDifference = (difference / targetVelocity) * 100.0;

        return percentDifference <= tolerancePercent;
    }

    // Get shooter state as string for telemetry
    public String getState() {
        if (currentPower > POWER_THRESHOLD) {
            return "Running";
        } else {
            return "Stopped";
        }
    }
}