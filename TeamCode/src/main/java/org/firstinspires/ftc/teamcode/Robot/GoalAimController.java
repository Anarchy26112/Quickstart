package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GoalAimController {

    private final Telemetry telemetry;
    private final Limelight limelight;

    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    private double goalX = GOAL_X;
    private double goalY = GOAL_Y;

    private boolean useVisionCorrection = false;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;
    private boolean usingFastProfile = false;

    private long lastUpdateNs = 0;

    public GoalAimController(Limelight limelight, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = limelight;
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

    public void setTargetBlue() {
        limelight.setTargetBlue();
    }

    public void setTargetRed() {
        limelight.setTargetRed();
    }

    public void setUseVisionCorrection(boolean useVisionCorrection) {
        this.useVisionCorrection = useVisionCorrection;
    }

    public void pollVision() {
        limelight.pollVision();
    }

    public void update() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double desiredHeadingRad = Math.atan2(dy, dx) + Math.toRadians(180);
        double errorRad = wrapAngleRad(desiredHeadingRad - robotHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        usingFastProfile = robotY < FAST_AIM_Y_THRESHOLD;

        double kP = usingFastProfile ? FAST_KP_TURN : PRECISE_KP_TURN;
        double kD = usingFastProfile ? FAST_KD_TURN : PRECISE_KD_TURN;
        double kS = usingFastProfile ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
        double deadbandDeg = usingFastProfile ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

        long nowNs = System.nanoTime();
        double dt = 0.02;
        if (lastUpdateNs != 0) {
            dt = (nowNs - lastUpdateNs) / 1_000_000_000.0;
        }
        lastUpdateNs = nowNs;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.1) dt = 0.1;

        double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double odomTurn = 0.0;
        if (Math.abs(errorDeg) > deadbandDeg) {
            odomTurn = (kP * errorDeg) + (kD * derivative) + Math.signum(errorDeg) * kS;
        }

        double visionTrim = 0.0;
        if (useVisionCorrection && limelight.isTargetVisible()) {
            visionTrim = limelight.getTurnPower();
        }

        turnPower = odomTurn + visionTrim;
        turnPower = clamp(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        if (telemetry != null) {
            telemetry.addData("Aim Goal", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addData(
                    "Aim Pose",
                    "(%.1f, %.1f, %.1fdeg)",
                    robotX, robotY, Math.toDegrees(robotHeadingRad)
            );
            telemetry.addData("Aim Err Deg", "%.2f", errorDeg);
            telemetry.addData("Aim Profile", usingFastProfile ? "FAST" : "PRECISE");
            telemetry.addData("Aim Use Vision", useVisionCorrection);
            telemetry.addData("Aim Turn", "%.3f", turnPower);
            telemetry.addData("Aim dt", "%.3f", dt);
        }
    }

    public double getTurnPower() {
        return turnPower;
    }

    public boolean isTargetVisible() {
        return limelight.isTargetVisible();
    }

    public boolean isSettled() {
        double shootDeadband = usingFastProfile
                ? FAST_SHOOT_READY_HEADING_ERROR_DEG
                : PRECISE_SHOOT_READY_HEADING_ERROR_DEG;

        return Math.abs(lastHeadingErrorDeg) <= shootDeadband;
    }

    public boolean isShootReady() {
        return isSettled();
    }

    public String getAimProfileName() {
        return usingFastProfile ? "FAST" : "PRECISE";
    }

    public boolean isUsingFastProfile() {
        return usingFastProfile;
    }

    public int getDetectedTagId() {
        return limelight.getDetectedTagId();
    }

    public double getFilteredRate() {
        return limelight.getFilteredRate();
    }

    public double getLastError() {
        return lastHeadingErrorDeg;
    }

    public double getHeadingErrorDeg() {
        return lastHeadingErrorDeg;
    }

    public boolean isFreshFrameThisLoop() {
        return limelight.isFreshFrameThisLoop();
    }

    public double getVisionTx() {
        return limelight.getTx();
    }

    public void sendTelemetry() {
        limelight.sendTelemetry();
    }

    private double wrapAngleRad(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public void reset() {
        lastHeadingErrorDeg = 0.0;
        turnPower = 0.0;
        usingFastProfile = false;
        lastUpdateNs = 0;
    }
}