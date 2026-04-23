package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GoalAimController {
    public enum AllianceColor { BLUE, RED }

    private final Telemetry telemetry;
    private final Follower follower;

    private double robotX, robotY, robotHeadingRad;
    private double robotVx = 0.0, robotVy = 0.0;
    private double robotAx = 0.0, robotAy = 0.0; // New: Acceleration components

    // Simple Low-Pass Filter coefficient (0.0 = no update, 1.0 = no filtering)
    private static final double ACCEL_LPF = 0.0;

    private double goalX = 0.0, goalY = 0.0;
    private AllianceColor allianceColor = AllianceColor.BLUE;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;
    private boolean usingFastProfile = false;
    private long lastUpdateNs = 0;

    private double baseDesiredHeadingDeg = 0.0;
    private double poseOffsetDeg = 0.0;
    private double odomDesiredHeadingRad = 0.0;
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate = 0.0;

    public GoalAimController(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    public void setRobotPose(double x, double y, double headingRad) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeadingRad = headingRad;
    }

    public void setRobotVelocity(double vx, double vy) {
        this.robotVx = vx;
        this.robotVy = vy;
    }

    /**
     * Updates acceleration with a low-pass filter to reduce sensor noise
     */
    public void setRobotAcceleration(double ax, double ay) {
        // this.robotAx = (ax * ACCEL_LPF) + (this.robotAx * (1.0 - ACCEL_LPF));
        // this.robotAy = (ay * ACCEL_LPF) + (this.robotAy * (1.0 - ACCEL_LPF));
    }

    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void setAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        setGoal(GOAL_X, (allianceColor == AllianceColor.RED) ? GOAL_Y_RED : GOAL_Y_BLUE);
    }

    private double getAllianceNormalizedY(double fieldY) {
        return (allianceColor == AllianceColor.RED) ? -fieldY : fieldY;
    }

    public void reset() {
        robotX = robotY = robotHeadingRad = robotVx = robotVy = robotAx = robotAy = 0.0;
        lastHeadingErrorDeg = turnPower = 0.0;
        usingFastProfile = false;
        lastUpdateNs = 0;
    }

    public void update(long nowMs, long nowNs) {
        double dt = (lastUpdateNs == 0) ? 0.02 : (nowNs - lastUpdateNs) * 1e-9;
        lastUpdateNs = nowNs;
        dt = clamp(dt, 0.001, 0.1);

        updateOdomAim();

        double errorRad = wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad);
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
    }

    private void updateOdomAim() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        if (SHOOT_ON_THE_MOVE_ENABLED && PROJECTILE_SPEED_INCHES_PER_SEC > 0) {
            double distanceToGoal = Math.hypot(dx, dy);
            double t = distanceToGoal / PROJECTILE_SPEED_INCHES_PER_SEC;

            // Second-order Kinematics: d = vt + 0.5 * a * t^2
            double virtualGoalX = goalX - (robotVx * t) - (0.5 * robotAx * t * t);
            double virtualGoalY = goalY - (robotVy * t) - (0.5 * robotAy * t * t);

            dx = virtualGoalX - robotX;
            dy = virtualGoalY - robotY;
        }

        double baseDesiredHeadingRad = Math.atan2(dy, dx) + Math.PI;
        baseDesiredHeadingDeg = Math.toDegrees(baseDesiredHeadingRad);

        double poseOffsetRad = Math.toRadians(getPoseBasedAimOffsetDeg(robotX, robotY));
        poseOffsetDeg = Math.toDegrees(poseOffsetRad);

        odomDesiredHeadingRad = baseDesiredHeadingRad + poseOffsetRad;
        odomDesiredHeadingDeg = Math.toDegrees(odomDesiredHeadingRad);
        odomErrorDegForGate = Math.toDegrees(wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad));
    }

    private double getPoseBasedAimOffsetDeg(double fieldX, double fieldY) {
        double y = Math.min(getAllianceNormalizedY(fieldY), -36.0);
        double offsetDeg;
        if (y <= -108.0) offsetDeg = Y_AIM_OFFSET_FAR_DEG;
        else if (y <= -60.0) offsetDeg = lerp(Y_AIM_OFFSET_FAR_DEG, Y_AIM_OFFSET_MID_DEG, (y + 108.0) / 48.0);
        else offsetDeg = lerp(Y_AIM_OFFSET_MID_DEG, Y_AIM_OFFSET_NEAR_DEG, (y + 60.0) / 24.0);

        return (allianceColor == AllianceColor.RED) ? -offsetDeg : offsetDeg;
    }

    public double getTurnPower() { return turnPower; }
    private static double lerp(double a, double b, double t) { return a + (b - a) * clamp(t, 0.0, 1.0); }
    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}