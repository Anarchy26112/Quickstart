package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {
    private final DcMotorEx shooterMotor;
    private final Telemetry telemetry;
    private double currentPower = 0.0;

    // Power constants
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01; // Minimum power to consider "running"

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooterMotor = hardwareMap.get(DcMotorEx.class, HW_SHOOTER);

        // Set direction
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to float for flywheels
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders for velocity monitoring
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set shooter power (0.0 to 1.0)
    public void setPower(double power) {
        currentPower = power;
        if (shooterMotor != null) {
            shooterMotor.setPower(power);
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

    // Get actual motor velocity (useful for monitoring)
    public double getVelocity() {
        if (shooterMotor != null) {
            return shooterMotor.getVelocity();
        }
        return 0.0;
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