package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final TelemetryManager telemetryM;
    private double currentPower = 0.0;
    private double currentRPower = 0.0;
    private double currentLPower = 0.0;

    // Power constants
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01; // Minimum power to consider "running"

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetryM = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

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

    // Set shooter power (0.0 to 1.0) for both motors
    public void setPower(double power) {
        currentPower = power;
        currentRPower = power;
        currentLPower = power;
        if (rightShooter != null && leftShooter != null) {
            rightShooter.setPower(power);
            leftShooter.setPower(power);
        }
    }

    // Set right shooter power independently
    public void setRPower(double power) {
        currentRPower = power;
        currentPower = (currentRPower + currentLPower) / 2.0;
        if (rightShooter != null) {
            rightShooter.setPower(power);
        }
    }

    // Set left shooter power independently
    public void setLPower(double power) {
        currentLPower = power;
        currentPower = (currentRPower + currentLPower) / 2.0;
        if (leftShooter != null) {
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

    // Check if shooter is running (either motor above threshold)
    public boolean isRunning() {
        return currentRPower > POWER_THRESHOLD || currentLPower > POWER_THRESHOLD;
    }

    // Check if shooter is stopped (both motors below threshold)
    public boolean isStopped() {
        return currentRPower <= POWER_THRESHOLD && currentLPower <= POWER_THRESHOLD;
    }

    // Get current average power setting
    public double getCurrentPower() {
        return currentPower;
    }

    // Get individual motor powers
    public double getCurrentRPower() {
        return currentRPower;
    }

    public double getCurrentLPower() {
        return currentLPower;
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

    // Get shooter state as string for telemetry
    public String getState() {
        if (currentRPower > POWER_THRESHOLD || currentLPower > POWER_THRESHOLD) {
            return "Running";
        } else {
            return "Stopped";
        }
    }
}