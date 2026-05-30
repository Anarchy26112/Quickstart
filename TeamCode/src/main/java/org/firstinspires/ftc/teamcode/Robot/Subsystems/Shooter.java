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

    private double targetVelocity = 0.0;
    private double currentRVel = 0.0;
    private double currentLVel = 0.0;

    private static final double STOP_VELOCITY = 0.0;
    private static final double TARGET_CHANGE_EPSILON = 30.0;

    // Lowered from 0.008 to 0.003 so small PID corrections are not ignored.
    private static final double WRITE_TOLERANCE = 0.003;
/*
    private static final double kV = 0.00034;
    private static final double kS = 0.02;
    private static final double kP_FAR = 0.0014;
    private static final double kP_NEAR = 0.0014;

 */
    private double kV = 0.000348;
    private double kS = 0.02;
    private double kP_FAR = 0.0014;
    private double kP_NEAR = 0.0009;
    private boolean shooterActive = false;
    private boolean isFarZone = true;

    private double lastWrittenRPower = 0.0;
    private double lastWrittenLPower = 0.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        configureMotor(rightShooter, DcMotor.Direction.FORWARD);
        configureMotor(leftShooter, DcMotor.Direction.REVERSE);
    }

    private void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    public void setRobotY(double y) {
        isFarZone = y <= AIM_FAR_ZONE_Y_THRESHOLD;
    }

    public void setVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void update(final double voltageComp) {
        final double currentTarget = targetVelocity;

        if (currentTarget < TARGET_CHANGE_EPSILON) {
            if (!shooterActive) return;

            targetVelocity = STOP_VELOCITY;
            currentRVel = 0.0;
            currentLVel = 0.0;
            shooterActive = false;

            writeMotorPowers(0.0, 0.0);
            return;
        }

        shooterActive = true;

        // Cache velocity reads to avoid multiple motor calls
        double rVel = rightShooter.getVelocity();
        double lVel = leftShooter.getVelocity();

        // Inline absolute value
        currentRVel = rVel < 0.0 ? -rVel : rVel;
        currentLVel = lVel < 0.0 ? -lVel : lVel;

        calculateAndSetPower(currentTarget, voltageComp);
    }

    private void calculateAndSetPower(final double target, final double voltageComp) {
        final double activeKP = isFarZone ? kP_FAR : kP_NEAR;
        final double feedForward = kV * target + kS;

        final double compFF = feedForward * voltageComp;
        final double compKP = activeKP * voltageComp;

        final double rError = target - currentRVel;
        final double lError = target - currentLVel;

        double rPower = compFF + compKP * rError;
        double lPower = compFF + compKP * lError;

        // Clamp inline
        if (rPower > 1.0) rPower = 1.0;
        else if (rPower < -1.0) rPower = -1.0;

        if (lPower > 1.0) lPower = 1.0;
        else if (lPower < -1.0) lPower = -1.0;

        writeMotorPowers(rPower, lPower);
    }

    private void writeMotorPowers(final double rightPower, final double leftPower) {
        if (shouldWritePower(rightPower, lastWrittenRPower)) {
            rightShooter.setPower(rightPower);
            lastWrittenRPower = rightPower;
        }

        if (shouldWritePower(leftPower, lastWrittenLPower)) {
            leftShooter.setPower(leftPower);
            lastWrittenLPower = leftPower;
        }
    }

    private boolean shouldWritePower(double newPower, double lastPower) {
        if (newPower == 0.0 && lastPower != 0.0) return true;

        if (newPower != 0.0) {
            double diff = newPower - lastPower;
            return diff > WRITE_TOLERANCE || diff < -WRITE_TOLERANCE;
        }

        return false;
    }

    public void stop() {
        targetVelocity = STOP_VELOCITY;
        currentRVel = 0.0;
        currentLVel = 0.0;
        shooterActive = false;
        writeMotorPowers(0.0, 0.0);
    }

    public double getAverageVelocity() {
        return 0.5 * (currentRVel + currentLVel);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRightVelocity() {
        return currentRVel;
    }

    public double getLeftVelocity() {
        return currentLVel;
    }

    public void telemetry() {
        if (telemetry == null) return;

        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Shooter Zone", isFarZone ? "Far" : "Near");

        telemetry.addData("Shooter Target", targetVelocity);
        telemetry.addData("Shooter Avg Vel", getAverageVelocity());
        telemetry.addData("Shooter R Vel", currentRVel);
        telemetry.addData("Shooter L Vel", currentLVel);
    }
}