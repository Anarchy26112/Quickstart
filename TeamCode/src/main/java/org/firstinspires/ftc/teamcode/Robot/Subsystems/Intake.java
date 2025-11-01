package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Intake {
    private DcMotor intake;
    private TelemetryManager telemetryM;
    private double currentPower = 0.0;

    // Power constants for consistency
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01; // Minimum power to consider "running"

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetryM = telemetry;

        intake = hardwareMap.get(DcMotor.class, HW_INTAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior for better control
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Don't use encoders for intake (simpler control)
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Start intaking at default power
    public void intake() {
        setPower(INTAKE_POWER);
    }

    // Start intaking at custom power
    public void intake(double customPower) {
        setPower(Math.abs(customPower)); // Ensure positive
    }

    // Start spitting out at default power
    public void spit() {
        setPower(SPIT_POWER);
    }

    // Start spitting at custom power
    public void spit(double customPower) {
        setPower(-Math.abs(customPower)); // Ensure negative
    }

    // Stop the intake
    public void stop() {
        setPower(STOP_POWER);
    }

    // Set custom intake power
    public void setPower(double power) {
        currentPower = power;
        if (intake != null) {
            intake.setPower(power);
        }
    }

    // Check if intake is running (any direction)
    public boolean isRunning() {
        return Math.abs(currentPower) > POWER_THRESHOLD;
    }

    // Get current power
    public double getCurrentPower() {
        return currentPower;
    }

    // Get intake state as string for telemetry
    public String getState() {
        if (currentPower > POWER_THRESHOLD) {
            return "Intaking";
        } else if (currentPower < -POWER_THRESHOLD) {
            return "Spitting";
        } else {
            return "Stopped";
        }
    }
}