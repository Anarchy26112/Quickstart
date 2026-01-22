package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private final Telemetry telemetry;

    // --- CAMERA CONFIGURATION ---
    private static final double CAMERA_TILT_DEGREES = 20.0;
    private static final double CAMERA_HEIGHT_INCHES = 10.5;
    private static final double APRILTAG_HEIGHT_INCHES = 29.5;

    // AprilTag detection results
    private LLResult result;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;

    // Calculated values for robot control
    public double horizontalDistance = 0.0;
    private double angleToTarget = 0.0;

    // --- PID CONTROL VARIABLES ---
    private double integral = 0.0;
    private double previousError = 0.0;
    private double lastReference = 0.0;  // Track setpoint changes
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean firstUpdate = true;

    // --- LOW-PASS FILTER VARIABLES ---
    private static final double FILTER_COEFFICIENT = 0.8;  // 0 < a < 1
    private double previousFilterEstimate = 0.0;
    private double currentFilterEstimate = 0.0;

    // --- PID SAFETY LIMITS ---
    private static final double MAX_INTEGRAL = 200.0;
    private static final double MIN_DT = 0.001;  // Minimum delta time to prevent spikes

    // Per-term output limits
    private static final double MAX_P_OUTPUT = 0.5;
    private static final double MAX_I_OUTPUT = 0.3;
    private static final double MAX_D_OUTPUT = 0.4;

    // --- STOP BAND ---
    private static final double DEADBAND_DEGREES = 1.0;

    // --- TARGET LOSS HANDLING (anti-jitter) ---
    private static final double TARGET_LOST_HOLD_SECONDS = 0.10;   // hold last power briefly
    private static final double TARGET_LOST_RESET_SECONDS = 0.25;  // reset PID after sustained loss
    private final ElapsedTime targetLossTimer = new ElapsedTime();
    private boolean hadTargetPreviously = false;

    // --- PID TERM STORAGE (for telemetry) ---
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastTurnPower = 0.0;
    private double lastRawPower = 0.0;

    // --- OFFSET / SETPOINT TELEMETRY ---
    private double lastDesiredTx = 0.0;   // degrees (0, +3, -3, or computed)
    private double lastError = 0.0;       // desiredTx - tx

    // --- TARGET FILTERING ---
    private final List<Integer> allowedTagIds = new ArrayList<>();

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        limelight.pipelineSwitch(4);
        limelight.start();

        pidTimer.reset();
        targetLossTimer.reset();
    }

    public void setTargetBlue() { setAllowedTags(20); }
    public void setTargetRed() { setAllowedTags(24); }
    public void setTargetMotif() { setAllowedTags(21, 22, 23); }

    /** Clear filters (track any visible tag). */
    public void trackAnyTag() { allowedTagIds.clear(); }

    private void setAllowedTags(Integer... tags) {
        allowedTagIds.clear();
        allowedTagIds.addAll(Arrays.asList(tags));
    }

    public void update() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult matchingTag = null;

            for (LLResultTypes.FiducialResult tag : fiducialResults) {
                int id = (int) tag.getFiducialId();
                if (allowedTagIds.isEmpty() || allowedTagIds.contains(id)) {
                    matchingTag = tag;
                    break; // Only one matching tag will be on field
                }
            }

            if (matchingTag != null) {
                targetVisible = true;
                detectedTagId = (int) matchingTag.getFiducialId();
                tx = matchingTag.getTargetXDegrees();
                ty = matchingTag.getTargetYDegrees();
                ta = matchingTag.getTargetArea();

                calculateDistanceAndAngle();

                // Target present this frame
                targetLossTimer.reset();
                hadTargetPreviously = true;
                return;
            }
        }

        // No valid matching tag this frame
        targetVisible = false;
        detectedTagId = -1;
        // Keep last tx/ty/ta for debugging; visibility flag controls behavior.
    }

    private void resetTargetDataHard() {
        targetVisible = false;
        detectedTagId = -1;
        tx = 0.0;
        ty = 0.0;
        ta = 0.0;
        horizontalDistance = 0.0;
        angleToTarget = 0.0;
        resetPID();
    }

    /** Helper method to calculate distance from ty angle */
    private double calculateDistance(double tyAngle) {
        double actualVerticalAngle = CAMERA_TILT_DEGREES + tyAngle;
        double heightDifference = APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        if (Math.abs(actualVerticalAngle) > 0.1) {
            return heightDifference / Math.tan(Math.toRadians(actualVerticalAngle));
        } else {
            return Double.MAX_VALUE;
        }
    }

    private void calculateDistanceAndAngle() {
        horizontalDistance = calculateDistance(ty);
        angleToTarget = tx;
    }

    // --- PID RESET METHOD ---
    private void resetPID() {
        integral = 0.0;
        previousError = 0.0;
        lastReference = 0.0;
        previousFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;
        firstUpdate = true;

        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
        lastTurnPower = 0.0;
        lastRawPower = 0.0;

        lastDesiredTx = 0.0;
        lastError = 0.0;

        pidTimer.reset();
    }


    private void resetDerivativeStateOnly(double currentError) {
        previousError = currentError;
        firstUpdate = true; // next loop becomes P-only
        previousFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;
    }

    // =========================
    // ALIGNMENT METHODS (Offsets)
    // =========================

    /** Normal centering (desired tx = 0). */
    public double getTurnPower() {
        return getTurnPowerToDesiredTx(0.0);
    }

    public double getTurnPowerTxOffsetPlus3() {
        return getTurnPowerToDesiredTx(3.0);
    }

    public double getTurnPowerTxOffsetMinus3() {
        return getTurnPowerToDesiredTx(-3.0);
    }

    public double getTurnPowerOffsetByInches(double inchesOffset) {
        if (!targetVisible || horizontalDistance < 1.0 || horizontalDistance == Double.MAX_VALUE) {
            return getTurnPowerToDesiredTx(0.0);
        }
        double angleOffsetDeg = Math.toDegrees(Math.atan2(inchesOffset, horizontalDistance));
        return getTurnPowerToDesiredTx(angleOffsetDeg);
    }

    private double getTurnPowerToDesiredTx(double desiredTxDegrees) {
        // --- 1) TARGET LOSS HANDLING (anti-jitter) ---
        if (!targetVisible) {
            if (!hadTargetPreviously) {
                resetPID();
                return 0.0;
            }

            double lostFor = targetLossTimer.seconds();

            // Hold briefly on flicker
            if (lostFor <= TARGET_LOST_HOLD_SECONDS) {
                return lastTurnPower;
            }

            // Reset after sustained loss
            if (lostFor >= TARGET_LOST_RESET_SECONDS) {
                resetPID();
                hadTargetPreviously = false;
                lastTurnPower = 0.0;
                lastRawPower = 0.0;
                return 0.0;
            }

            // Decay smoothly between hold and reset
            double t = (lostFor - TARGET_LOST_HOLD_SECONDS) /
                    (TARGET_LOST_RESET_SECONDS - TARGET_LOST_HOLD_SECONDS);
            t = clamp(t, 0.0, 1.0);
            lastTurnPower = lastTurnPower * (1.0 - t);
            return lastTurnPower;
        }

        // --- 2) TIME STEP ---
        if (pidTimer.seconds() < 0.005) {
            return lastTurnPower;
        }
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // --- 3) ERROR
        // If desiredTx is +3, we want the robot to stop when tx ≈ +3
        double reference = desiredTxDegrees;
        double error = reference - angleToTarget; // (desiredTx - tx)
        lastError = error;

        // --- 4) DERIVATIVE KICK PROTECTION (setpoint change) ---
        if (Math.abs(reference - lastReference) > 0.1) {
            resetDerivativeStateOnly(error);
        }
        lastReference = reference;
        lastDesiredTx = reference;

        // --- 5) DEADBAND ---
        if (Math.abs(error) < DEADBAND_DEGREES) {
            lastTurnPower = 0.0;
            lastRawPower = 0.0;
            previousError = error;    // keep derivative sane
            return 0.0;
        }


        // --- 6) FIRST LOOP (P-only) ---
        if (firstUpdate) {
            firstUpdate = false;
            previousError = error;

            double simplePower = error * Kp_TURN;
            lastPTerm = simplePower;
            lastITerm = 0.0;
            lastDTerm = 0.0;
            lastRawPower = simplePower;

            if (Math.abs(simplePower) < MIN_TURN_POWER) {
                simplePower = Math.signum(simplePower) * MIN_TURN_POWER;
            }

            lastTurnPower = clamp(simplePower, -1.0, 1.0);
            return lastTurnPower;
        }

        // ===== PROPORTIONAL =====
        double p = error * Kp_TURN;
        p = clamp(p, -MAX_P_OUTPUT, MAX_P_OUTPUT);

        // ===== INTEGRAL =====
        integral += error * dt;
        integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        double i = integral * Ki_TURN;
        i = clamp(i, -MAX_I_OUTPUT, MAX_I_OUTPUT);

        // ===== DERIVATIVE =====
        double errorChange = error - previousError;

        currentFilterEstimate = (FILTER_COEFFICIENT * previousFilterEstimate) +
                (1.0 - FILTER_COEFFICIENT) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = 0.0;
        if (dt > MIN_DT) {
            derivative = currentFilterEstimate / dt;
        }

        double d = derivative * Kd_TURN;
        d = clamp(d, -MAX_D_OUTPUT, MAX_D_OUTPUT);

        lastPTerm = p;
        lastITerm = i;
        lastDTerm = d;

        previousError = error;

        // ===== SUM =====
        double turnPower = p + i + d;
        lastRawPower = turnPower;

        // Stiction
        if (Math.abs(turnPower) < MIN_TURN_POWER) {
            turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
        }

        turnPower = clamp(turnPower, -1.0, 1.0);
        lastTurnPower = turnPower;

        return turnPower;
    }

    public double getTurnPowerSmartOffsetByDistance(double switchDistanceInches, double farOffsetDeg) {
        // If we can't trust distance, just do normal centering
        if (!targetVisible || horizontalDistance == Double.MAX_VALUE || horizontalDistance < 1.0) {
            return getTurnPowerToDesiredTx(0.0);
        }

        double desiredTx = (horizontalDistance >= switchDistanceInches) ? farOffsetDeg : 0.0;
        return getTurnPowerToDesiredTx(desiredTx);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // --- GETTERS ---
    public boolean isTargetVisible() { return targetVisible; }
    public int getDetectedTagId() { return detectedTagId; }
    public double getAngleToTarget() { return angleToTarget; }
    public double getHorizontalDistance() { return horizontalDistance; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double getIntegral() { return integral; }
    public double getPreviousError() { return previousError; }


    public boolean isCenteredOnTarget(double toleranceDegrees) {
        return targetVisible && Math.abs(lastError) <= toleranceDegrees;
    }

    public void displayTelemetry() {
        telemetry.addData("Limelight", "Status: %s", limelight.isConnected() ? "Connected" : "Disconnected");

        String lookingFor = allowedTagIds.isEmpty() ? "ANY" : allowedTagIds.toString();
        telemetry.addData("Target Filter", lookingFor);
        telemetry.addData("Target Visible", targetVisible);

        if (targetVisible) {
            telemetry.addData("AprilTag ID", detectedTagId);
            telemetry.addData("Distance", "%.2f in", horizontalDistance);

            telemetry.addData("--- Aim Debug ---", "");
            telemetry.addData("tx (measured)", "%.2f deg", tx);
            telemetry.addData("desired tx", "%.2f deg", lastDesiredTx);
            telemetry.addData("error (des-tx)", "%.2f deg", lastError);

            telemetry.addData("--- PID Debug ---", "");
            telemetry.addData("Raw Power", "%.3f", lastRawPower);
            telemetry.addData("Turn Power (w/ threshold)", "%.3f", lastTurnPower);
            telemetry.addData("P term", "%.3f", lastPTerm);
            telemetry.addData("I term", "%.3f (int: %.1f)", lastITerm, integral);
            telemetry.addData("D term", "%.3f (filter: %.3f)", lastDTerm, currentFilterEstimate);
        }
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
