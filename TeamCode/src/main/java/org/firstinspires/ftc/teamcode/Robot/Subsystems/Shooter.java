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

    // Prevent excessive motor writes for tiny power changes.
    private static final double WRITE_TOLERANCE = 0.003;

    // Feedforward values
    private double kV = 0.000360;
    private double kS = 0.0;
    private double kP_FAR = 0.002;
    private double kP_NEAR = 0.0014;
    private double kD_FAR = 0.00;
    private double kD_NEAR = 0.0;

    // Derivative safety/filtering
    private static final double MIN_DT_SEC = 0.0001;
    private static final double MAX_DT_SEC = 0.1;
    private static final double D_FILTER_TAU = 0.02;
    private static final double MAX_D_POWER = 0.07;
    private static final double D_TARGET_RESET_EPSILON = 250.0;

    private boolean shooterActive = false;
    private boolean isFarZone = true;

    private double lastWrittenRPower = 0.0;
    private double lastWrittenLPower = 0.0;

    // Derivative state
    private double lastRVelForD = 0.0;
    private double lastLVelForD = 0.0;
    private double filteredRAccel = 0.0;
    private double filteredLAccel = 0.0;
    private double lastTargetForD = 0.0;
    private boolean derivativeReady = false;

    // Ready state
    private boolean readyToShoot = false;

    // Telemetry/debug
    private double lastRPower = 0.0;
    private double lastLPower = 0.0;
    private double lastRPTerm = 0.0;
    private double lastLPTerm = 0.0;
    private double lastRDTerm = 0.0;
    private double lastLDTerm = 0.0;
    private double lastFeedForward = 0.0; // stores voltage-compensated FF

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

    public void update(final double voltageComp, double dtSec) {
        final double currentTarget = targetVelocity;

        if (dtSec < MIN_DT_SEC) {
            dtSec = MIN_DT_SEC;
        } else if (dtSec > MAX_DT_SEC) {
            dtSec = MAX_DT_SEC;
        }

        if (currentTarget < TARGET_CHANGE_EPSILON) {
            if (!shooterActive) {
                readyToShoot = false;
                return;
            }

            targetVelocity = STOP_VELOCITY;
            currentRVel = 0.0;
            currentLVel = 0.0;
            shooterActive = false;
            readyToShoot = false;

            resetDerivative();
            writeMotorPowers(0.0, 0.0);
            return;
        }

        shooterActive = true;

        // Read velocity once per loop.
        double rVel = rightShooter.getVelocity();
        double lVel = leftShooter.getVelocity();

        currentRVel = rVel < 0.0 ? -rVel : rVel;
        currentLVel = lVel < 0.0 ? -lVel : lVel;

        calculateAndSetPower(currentTarget, voltageComp, dtSec);

        // No tolerance check anymore.
        // Shooter is ready whenever it is active and has a valid target.
        updateReadyState(currentTarget);
    }

    private void calculateAndSetPower(
            final double target,
            final double voltageComp,
            final double dtSec
    ) {
        final double activeKP = isFarZone ? kP_FAR : kP_NEAR;
        final double activeKD = isFarZone ? kD_FAR : kD_NEAR;

        final double feedForward = kV * target + kS;

        final double rError = target - currentRVel;
        final double lError = target - currentLVel;

        double targetChange = target - lastTargetForD;
        if (targetChange < 0.0) {
            targetChange = -targetChange;
        }

        if (!derivativeReady || targetChange > D_TARGET_RESET_EPSILON) {
            filteredRAccel = 0.0;
            filteredLAccel = 0.0;

            lastRVelForD = currentRVel;
            lastLVelForD = currentLVel;
            lastTargetForD = target;

            derivativeReady = true;
        }

        final double rawRAccel = (currentRVel - lastRVelForD) / dtSec;
        final double rawLAccel = (currentLVel - lastLVelForD) / dtSec;

        final double alpha = D_FILTER_TAU / (D_FILTER_TAU + dtSec);

        filteredRAccel = alpha * filteredRAccel + (1.0 - alpha) * rawRAccel;
        filteredLAccel = alpha * filteredLAccel + (1.0 - alpha) * rawLAccel;

        double rDTerm = -activeKD * filteredRAccel;
        double lDTerm = -activeKD * filteredLAccel;

        if (rDTerm > MAX_D_POWER) {
            rDTerm = MAX_D_POWER;
        } else if (rDTerm < -MAX_D_POWER) {
            rDTerm = -MAX_D_POWER;
        }

        if (lDTerm > MAX_D_POWER) {
            lDTerm = MAX_D_POWER;
        } else if (lDTerm < -MAX_D_POWER) {
            lDTerm = -MAX_D_POWER;
        }

        final double rPTerm = activeKP * rError;
        final double lPTerm = activeKP * lError;

        // Voltage compensation is applied ONLY to the feedforward term.
        final double feedForwardComp = feedForward * voltageComp;

        double rPower = feedForwardComp + rPTerm + rDTerm;
        double lPower = feedForwardComp + lPTerm + lDTerm;

        if (rPower > 1.0) {
            rPower = 1.0;
        } else if (rPower < -1.0) {
            rPower = -1.0;
        }

        if (lPower > 1.0) {
            lPower = 1.0;
        } else if (lPower < -1.0) {
            lPower = -1.0;
        }

        lastRVelForD = currentRVel;
        lastLVelForD = currentLVel;
        lastTargetForD = target;

        lastFeedForward = feedForwardComp;
        lastRPTerm = rPTerm;
        lastLPTerm = lPTerm;
        lastRDTerm = rDTerm;
        lastLDTerm = lDTerm;
        lastRPower = rPower;
        lastLPower = lPower;

        writeMotorPowers(rPower, lPower);
    }

    private void updateReadyState(final double target) {
        readyToShoot = shooterActive && target >= TARGET_CHANGE_EPSILON;
    }

    private void resetDerivative() {
        lastRVelForD = 0.0;
        lastLVelForD = 0.0;
        filteredRAccel = 0.0;
        filteredLAccel = 0.0;
        lastTargetForD = 0.0;
        derivativeReady = false;

        readyToShoot = false;

        lastRPTerm = 0.0;
        lastLPTerm = 0.0;
        lastRDTerm = 0.0;
        lastLDTerm = 0.0;
        lastFeedForward = 0.0;
        lastRPower = 0.0;
        lastLPower = 0.0;
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
        if (newPower == 0.0 && lastPower != 0.0) {
            return true;
        }

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
        readyToShoot = false;

        resetDerivative();
        writeMotorPowers(0.0, 0.0);
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
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
        telemetry.addData("Shooter Ready", readyToShoot);
        telemetry.addData("Shooter Zone", isFarZone ? "Far" : "Near");

        telemetry.addData("Shooter Target", targetVelocity);
        telemetry.addData("Shooter Avg Vel", getAverageVelocity());
        telemetry.addData("Shooter R Vel", currentRVel);
        telemetry.addData("Shooter L Vel", currentLVel);

        telemetry.addData("Shooter R Error", targetVelocity - currentRVel);
        telemetry.addData("Shooter L Error", targetVelocity - currentLVel);

        telemetry.addData("Shooter FF (comp)", lastFeedForward);
        telemetry.addData("Shooter R P", lastRPTerm);
        telemetry.addData("Shooter L P", lastLPTerm);
        telemetry.addData("Shooter R D", lastRDTerm);
        telemetry.addData("Shooter L D", lastLDTerm);

        telemetry.addData("Shooter R Power", lastRPower);
        telemetry.addData("Shooter L Power", lastLPower);
    }
}