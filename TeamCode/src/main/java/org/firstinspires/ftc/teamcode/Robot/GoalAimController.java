package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;

public class GoalAimController {
    public enum AllianceColor { BLUE, RED }

    private final Telemetry telemetry;
    private final Follower follower;

    private double robotX, robotY, robotHeadingRad;
    private double robotVx = 0.0, robotVy = 0.0;
    private double robotAx = 0.0, robotAy = 0.0;

    private double goalX = 0.0, goalY = 0.0;
    private AllianceColor allianceColor = AllianceColor.BLUE;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;
    private boolean usingFastProfile = false;
    private boolean usingFarGoal = false;
    private long lastUpdateNs = 0;

    private double baseDesiredHeadingDeg = 0.0;
    private double odomDesiredHeadingRad = 0.0;
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate = 0.0;

    public GoalAimController(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;

        setAlliance(AllianceColor.BLUE);
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
     * Updates acceleration with a low-pass filter to reduce sensor noise.
     * Currently unused.
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
        updateGoalForCurrentZone();
    }

    private Alliance getMirrorAlliance() {
        return allianceColor == AllianceColor.BLUE ? Alliance.BLUE : Alliance.RED;
    }

    private void updateGoalForCurrentZone() {
        usingFarGoal = robotY < AIM_FAR_ZONE_Y_THRESHOLD;

        double blueGoalX = usingFarGoal ? GOAL_FAR_X_BLUE : GOAL_CLOSE_X_BLUE;
        double blueGoalY = usingFarGoal ? GOAL_FAR_Y_BLUE : GOAL_CLOSE_Y_BLUE;

        Pose goalPose = FieldMirror.pose(
                getMirrorAlliance(),
                blueGoalX,
                blueGoalY,
                0.0
        );

        setGoal(goalPose.getX(), goalPose.getY());
    }

    public void reset() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeadingRad = 0.0;
        robotVx = 0.0;
        robotVy = 0.0;
        robotAx = 0.0;
        robotAy = 0.0;

        lastHeadingErrorDeg = 0.0;
        turnPower = 0.0;
        usingFastProfile = false;
        usingFarGoal = false;
        lastUpdateNs = 0;

        updateGoalForCurrentZone();
    }

    public void update(long nowMs, long nowNs) {
        double dt = lastUpdateNs == 0
                ? 0.02
                : (nowNs - lastUpdateNs) * 1e-9;

        lastUpdateNs = nowNs;
        dt = clamp(dt, 0.001, 0.1);

        updateGoalForCurrentZone();
        updateOdomAim();

        double errorRad = wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        usingFastProfile = robotY > FAST_AIM_Y_THRESHOLD;

        double kP = usingFastProfile ? FAST_KP_TURN : PRECISE_KP_TURN;
        double kD = usingFastProfile ? FAST_KD_TURN : PRECISE_KD_TURN;
        double kS = usingFastProfile ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
        double deadbandDeg = usingFastProfile ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

        double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double odomOut = 0.0;

        if (Math.abs(errorDeg) > deadbandDeg) {
            odomOut = (kP * errorDeg)
                    + (kD * derivative)
                    + Math.signum(errorDeg) * kS;
        }

        turnPower = clamp(odomOut, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        telemetry.addData("Aim Alliance", allianceColor);
        telemetry.addData("Aim Zone", usingFarGoal ? "FAR" : "CLOSE");
        telemetry.addData("Aim Goal X", goalX);
        telemetry.addData("Aim Goal Y", goalY);
        telemetry.addData("Aim Desired Deg", odomDesiredHeadingDeg);
        telemetry.addData("Aim Error Deg", errorDeg);
        telemetry.addData("Aim Turn Power", turnPower);
        telemetry.addData("Aim Fast Profile", usingFastProfile);
    }

    private void updateOdomAim() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double baseDesiredHeadingRad = Math.atan2(dy, dx) + Math.PI;

        baseDesiredHeadingDeg = Math.toDegrees(baseDesiredHeadingRad);
        odomDesiredHeadingRad = baseDesiredHeadingRad;
        odomDesiredHeadingDeg = Math.toDegrees(odomDesiredHeadingRad);
        odomErrorDegForGate = Math.toDegrees(
                wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad)
        );
    }

    public double getTurnPower() {
        return turnPower;
    }

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
    }

    public boolean isUsingFarGoal() {
        return usingFarGoal;
    }

    public double getOdomDesiredHeadingRad() {
        return odomDesiredHeadingRad;
    }

    public double getOdomDesiredHeadingDeg() {
        return odomDesiredHeadingDeg;
    }

    public double getOdomErrorDegForGate() {
        return odomErrorDegForGate;
    }

    public boolean isUsingFastProfile() {
        return usingFastProfile;
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }

        while (a < -Math.PI) {
            a += 2.0 * Math.PI;
        }

        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}