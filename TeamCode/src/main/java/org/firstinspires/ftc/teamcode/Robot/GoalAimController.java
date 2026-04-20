package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GoalAimController {
    public enum AllianceColor {
        BLUE,
        RED
    }

    private final Telemetry telemetry;
    private final Follower follower;

    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    private double goalX = 0.0;
    private double goalY = 0.0;
    private AllianceColor allianceColor = AllianceColor.BLUE;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;
    private boolean usingFastProfile = false;

    private long lastUpdateNs = 0;

    private double baseDesiredHeadingDeg = 0.0;
    private double poseOffsetDeg = 0.0;
    private double odomDesiredHeadingRad = 0.0; // Stored here now to avoid object creation
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate = 0.0;

    private static final double HEADING_RATE_ALPHA = 0.25;
    private double filteredHeadingRateDegPerSec = 0.0;
    private double lastRobotHeadingDegForRate = 0.0;
    private boolean haveLastHeadingForRate = false;

    private long shootReadySinceMs = 0;
    private boolean shootReadyRaw = false;
    private boolean shootReadyLatched = false;
    private String shootBlockReason = "INIT";

    public GoalAimController(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    public void setRobotPose(double x, double y, double headingRad) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeadingRad = headingRad;
    }

    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void setAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        if (allianceColor == AllianceColor.RED) {
            setGoal(GOAL_X, GOAL_Y_RED);
        } else {
            setGoal(GOAL_X, GOAL_Y_BLUE);
        }
    }

    private double getAllianceYSign() {
        return (allianceColor == AllianceColor.RED) ? -1.0 : 1.0;
    }

    private double getAllianceNormalizedY(double fieldY) {
        return fieldY * getAllianceYSign();
    }

    public void reset() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeadingRad = 0.0;

        lastHeadingErrorDeg = 0.0;
        turnPower = 0.0;
        usingFastProfile = false;
        lastUpdateNs = 0;

        baseDesiredHeadingDeg = 0.0;
        poseOffsetDeg = 0.0;
        odomDesiredHeadingRad = 0.0;
        odomDesiredHeadingDeg = 0.0;
        odomErrorDegForGate = 0.0;

        filteredHeadingRateDegPerSec = 0.0;
        lastRobotHeadingDegForRate = 0.0;
        haveLastHeadingForRate = false;

        shootReadySinceMs = 0;
        shootReadyRaw = false;
        shootReadyLatched = false;
        shootBlockReason = "RESET";
    }

    public void update(long nowMs, long nowNs) {
        double dt = 0.02;
        if (lastUpdateNs != 0) {
            dt = (nowNs - lastUpdateNs) * 1e-9;
        }
        lastUpdateNs = nowNs;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.1) dt = 0.1;

        updateOdomAim(); // Updates the class variable instead of creating an object
        updateHeadingRate(dt);

        double effectiveHeadingRad = robotHeadingRad;
        double errorRad = wrapAngleRad(odomDesiredHeadingRad - effectiveHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        double normalizedY = getAllianceNormalizedY(robotY);
        usingFastProfile = normalizedY < FAST_AIM_Y_THRESHOLD;

        double kP = usingFastProfile ? FAST_KP_TURN : PRECISE_KP_TURN;
        double kD = usingFastProfile ? FAST_KD_TURN : PRECISE_KD_TURN;
        double kS = usingFastProfile ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
        double deadbandDeg = usingFastProfile ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

        double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double odomOut = 0.0;
        if (Math.abs(errorDeg) > deadbandDeg) {
            odomOut = (kP * errorDeg) + (kD * derivative) + Math.signum(errorDeg) * kS;
        }

        turnPower = clamp(odomOut, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        updateShootReady(nowMs);

        if (telemetry != null) {
            telemetry.addData("Aim Goal", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addData("Aim Pose", "(%.1f, %.1f, %.1fdeg)",
                    robotX, robotY, Math.toDegrees(robotHeadingRad));
            telemetry.addData("Aim Base Heading", "%.2f", baseDesiredHeadingDeg);
            telemetry.addData("Aim Pose Offset", "%.2f", poseOffsetDeg);
            telemetry.addData("Aim Odom Heading", "%.2f", odomDesiredHeadingDeg);
            telemetry.addData("Aim Odom Err", "%.2f", odomErrorDegForGate);
            telemetry.addData("Aim Profile", usingFastProfile ? "FAST" : "PRECISE");
            telemetry.addData("Aim Turn", "%.3f", turnPower);
            telemetry.addData("Aim dt", "%.3f", dt);
            telemetry.addData("Shoot Ready Raw", shootReadyRaw);
            telemetry.addData("Shoot Ready Latched", shootReadyLatched);
            telemetry.addData("Shoot Block", shootBlockReason);
        }
    }

    private void updateOdomAim() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double baseDesiredHeadingRad = Math.atan2(dy, dx) + Math.PI;
        baseDesiredHeadingDeg = Math.toDegrees(baseDesiredHeadingRad);

        double poseOffsetRad = Math.toRadians(getPoseBasedAimOffsetDeg(robotX, robotY));
        poseOffsetDeg = Math.toDegrees(poseOffsetRad);

        odomDesiredHeadingRad = baseDesiredHeadingRad + poseOffsetRad;
        odomDesiredHeadingDeg = Math.toDegrees(odomDesiredHeadingRad);

        double odomErrorRad = wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad);
        odomErrorDegForGate = Math.toDegrees(odomErrorRad);
    }

    private void updateHeadingRate(double dt) {
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        if (!haveLastHeadingForRate) {
            lastRobotHeadingDegForRate = robotHeadingDeg;
            haveLastHeadingForRate = true;
            filteredHeadingRateDegPerSec = 0.0;
            return;
        }

        double deltaDeg = wrapAngleDeg(robotHeadingDeg - lastRobotHeadingDegForRate);
        double rawRateDegPerSec = deltaDeg / dt;

        filteredHeadingRateDegPerSec +=
                HEADING_RATE_ALPHA * (rawRateDegPerSec - filteredHeadingRateDegPerSec);

        lastRobotHeadingDegForRate = robotHeadingDeg;
    }

    private void updateShootReady(long nowMs) {
        double allowedHeadingError = usingFastProfile
                ? FAST_SHOOT_READY_HEADING_ERROR_DEG
                : PRECISE_SHOOT_READY_HEADING_ERROR_DEG;

        double allowedHeadingRate = usingFastProfile
                ? FAST_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC
                : PRECISE_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC;

        shootReadyRaw =
                Math.abs(lastHeadingErrorDeg) <= allowedHeadingError
                        && Math.abs(filteredHeadingRateDegPerSec) <= allowedHeadingRate;

        if (shootReadyRaw) {
            if (shootReadySinceMs == 0) {
                shootReadySinceMs = nowMs;
            }
            shootReadyLatched = (nowMs - shootReadySinceMs) >= SHOOT_READY_SETTLE_MS;
            shootBlockReason = shootReadyLatched ? "READY" : "SETTLING";
        } else {
            shootReadySinceMs = 0;
            shootReadyLatched = false;

            if (Math.abs(lastHeadingErrorDeg) > allowedHeadingError) {
                shootBlockReason = "HEADING";
            } else {
                shootBlockReason = "RATE";
            }
        }
    }

    private double getPoseBasedAimOffsetDeg(double fieldX, double fieldY) {
        double y = getAllianceNormalizedY(fieldY);
        y = Math.min(y, -36.0);

        double offsetDeg;
        if (y <= -108.0) {
            offsetDeg = Y_AIM_OFFSET_FAR_DEG;
        } else if (y <= -60.0) {
            offsetDeg = lerp(Y_AIM_OFFSET_FAR_DEG, Y_AIM_OFFSET_MID_DEG, (y + 108.0) / 48.0);
        } else {
            offsetDeg = lerp(Y_AIM_OFFSET_MID_DEG, Y_AIM_OFFSET_NEAR_DEG, (y + 60.0) / 24.0);
        }

        return (allianceColor == AllianceColor.RED) ? -offsetDeg : offsetDeg;
    }

    public boolean isShootReady() { return shootReadyRaw; }
    public boolean isShootReadyLatched() { return shootReadyLatched; }
    public String getShootBlockReason() { return shootBlockReason; }
    public double getTurnPower() { return turnPower; }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * clamp(t, 0.0, 1.0);
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapAngleDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}