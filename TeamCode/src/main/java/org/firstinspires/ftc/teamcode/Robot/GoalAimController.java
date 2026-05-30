package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class GoalAimController {

    public enum AllianceColor { BLUE, RED }

    private final Telemetry telemetry;

    private double robotX, robotY, robotHeadingRad;
    private double goalX = 0.0, goalY = 0.0;

    private double closeGoalX = 0.0, closeGoalY = 0.0;
    private double farGoalX   = 0.0, farGoalY   = 0.0;

    // Separate target used only for shooter velocity distance lookup.
    // This lets aim use close/far goals while shooter lookup uses one fixed point.
    private double shooterTargetX = 0.0, shooterTargetY = 141.5;

    private AllianceColor allianceColor = AllianceColor.BLUE;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower           = 0.0;

    private boolean usingFastProfile = false;
    private boolean usingFarGoal     = false;
    private boolean goalInitialized  = false;
    private boolean lastUsingFarGoal = false;

    private long lastUpdateNs = 0L;

    private double odomDesiredHeadingRad = 0.0;
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate   = 0.0;
    private double lastErrorDegForTelemetry = 0.0;

    private double lastRobotX = -999.0;
    private double lastRobotY = -999.0;
    private boolean aimDirty  = true;

    private static final double PI         = Math.PI;
    private static final double TWO_PI     = 2.0 * Math.PI;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private static final double MIN_AIM_DT                  = 0.001;
    private static final double MAX_AIM_DT                  = 0.1;
    private static final double POSITION_CHANGE_THRESHOLD_SQ = 0.0225;

    // Cached control PID constants.
    // Start with precise values because far zone uses precise profile.
    private double cachedKP       = PRECISE_KP_TURN;
    private double cachedKD       = PRECISE_KD_TURN;
    private double cachedKS       = PRECISE_kS_VOLTAGE_COMP;
    private double cachedDeadband = PRECISE_ERROR_DEADBAND_DEG;

    private boolean cachedIsFast = false;
    private boolean cachedProfileInitialized = false;

    public GoalAimController(Telemetry telemetry) {
        this.telemetry = telemetry;
        setAlliance(AllianceColor.BLUE);
    }

    public void setRobotPose(double x, double y, double headingRad) {
        robotX = x;
        robotY = y;
        robotHeadingRad = headingRad;
    }

    public void setGoal(double x, double y) {
        goalX = x;
        goalY = y;
        goalInitialized = true;
        aimDirty = true;
    }

    public void setAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        final Alliance mirrorAlliance =
                allianceColor == AllianceColor.BLUE ? Alliance.BLUE : Alliance.RED;

        final Pose closeGoalPose = FieldMirror.pose(
                mirrorAlliance,
                GOAL_CLOSE_X_BLUE,
                GOAL_CLOSE_Y_BLUE,
                0.0
        );

        final Pose farGoalPose = FieldMirror.pose(
                mirrorAlliance,
                GOAL_FAR_X_BLUE,
                GOAL_FAR_Y_BLUE,
                0.0
        );

        final Pose shooterTargetPose = FieldMirror.pose(
                mirrorAlliance,
                SHOOTER_TARGET_X_BLUE,
                SHOOTER_TARGET_Y_BLUE,
                0.0
        );

        closeGoalX = closeGoalPose.getX();
        closeGoalY = closeGoalPose.getY();

        farGoalX = farGoalPose.getX();
        farGoalY = farGoalPose.getY();

        shooterTargetX = shooterTargetPose.getX();
        shooterTargetY = shooterTargetPose.getY();

        goalInitialized = false;
        aimDirty = true;

        updateGoalForCurrentZone(robotY < AIM_FAR_ZONE_Y_THRESHOLD);
    }

    private void updateGoalForCurrentZone(boolean isFar) {
        usingFarGoal     = isFar;
        lastUsingFarGoal = isFar;
        goalInitialized  = true;

        if (isFar) {
            goalX = farGoalX;
            goalY = farGoalY;
        } else {
            goalX = closeGoalX;
            goalY = closeGoalY;
        }

        aimDirty = true;
    }

    public void reset() {
        robotX = robotY = robotHeadingRad = 0.0;

        lastHeadingErrorDeg = 0.0;
        lastErrorDegForTelemetry = 0.0;
        turnPower = 0.0;

        usingFastProfile = false;
        usingFarGoal     = false;
        lastUsingFarGoal = false;
        goalInitialized  = false;

        lastUpdateNs = 0L;

        lastRobotX = -999.0;
        lastRobotY = -999.0;

        odomDesiredHeadingRad = 0.0;
        odomDesiredHeadingDeg = 0.0;
        odomErrorDegForGate   = 0.0;

        aimDirty = true;

        cachedKP       = PRECISE_KP_TURN;
        cachedKD       = PRECISE_KD_TURN;
        cachedKS       = PRECISE_kS_VOLTAGE_COMP;
        cachedDeadband = PRECISE_ERROR_DEADBAND_DEG;

        cachedIsFast = false;
        cachedProfileInitialized = false;

        setAlliance(allianceColor);
    }

    public void forceIdle(long nowNs) {
        turnPower           = 0.0;
        lastHeadingErrorDeg = 0.0;
        lastUpdateNs        = nowNs;
    }

    public void updateActive(
            final double currentRobotX,
            final double currentRobotY,
            final double currentRobotHeading,
            final long nowNs,
            double dt
    ) {
        robotX          = currentRobotX;
        robotY          = currentRobotY;
        robotHeadingRad = currentRobotHeading;
        lastUpdateNs    = nowNs;

        if      (dt < MIN_AIM_DT) dt = MIN_AIM_DT;
        else if (dt > MAX_AIM_DT) dt = MAX_AIM_DT;

        // Far zone: robotY < threshold.
        // Close zone: robotY >= threshold.
        final boolean isFar = currentRobotY < AIM_FAR_ZONE_Y_THRESHOLD;

        if (isFar != lastUsingFarGoal || !goalInitialized) {
            updateGoalForCurrentZone(isFar);

            // Avoid derivative kick when the target goal jumps.
            lastHeadingErrorDeg = 0.0;
        }

        updateOdomAim(currentRobotX, currentRobotY);

        final double errorRad = wrapPiFast(odomDesiredHeadingRad - currentRobotHeading);
        final double errorDeg = errorRad * RAD_TO_DEG;

        lastErrorDegForTelemetry = errorDeg;
        odomErrorDegForGate      = errorDeg;

        /*
         * Profile choice:
         *
         * Far zone   = precise/passive profile.
         * Close zone = fast/aggressive profile.
         *
         * Tie the profile directly to isFar so the goal zone and gain profile
         * can never disagree at the threshold.
         */
        final boolean useFast = !isFar;

        if (!cachedProfileInitialized || useFast != cachedIsFast) {
            cachedProfileInitialized = true;
            cachedIsFast = useFast;

            if (useFast) {
                cachedKP       = FAST_KP_TURN;
                cachedKD       = FAST_KD_TURN;
                cachedKS       = FAST_kS_VOLTAGE_COMP;
                cachedDeadband = FAST_ERROR_DEADBAND_DEG;
            } else {
                cachedKP       = PRECISE_KP_TURN;
                cachedKD       = PRECISE_KD_TURN;
                cachedKS       = PRECISE_kS_VOLTAGE_COMP;
                cachedDeadband = PRECISE_ERROR_DEADBAND_DEG;
            }

            usingFastProfile = useFast;

            // Avoid derivative kick when the profile changes.
            lastHeadingErrorDeg = errorDeg;
        }

        final double absError = errorDeg < 0.0 ? -errorDeg : errorDeg;

        if (absError <= cachedDeadband) {
            turnPower           = 0.0;
            lastHeadingErrorDeg = errorDeg;
            return;
        }

        final double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double output = cachedKP * errorDeg + cachedKD * derivative;
        output += errorDeg > 0.0 ? cachedKS : -cachedKS;

        if      (output >  MAX_AUTO_TURN) turnPower =  MAX_AUTO_TURN;
        else if (output < -MAX_AUTO_TURN) turnPower = -MAX_AUTO_TURN;
        else                              turnPower =  output;
    }

    private void updateOdomAim(final double rx, final double ry) {
        final double dxMove = rx - lastRobotX;
        final double dyMove = ry - lastRobotY;

        if (aimDirty || (dxMove * dxMove + dyMove * dyMove) > POSITION_CHANGE_THRESHOLD_SQ) {
            odomDesiredHeadingRad = Math.atan2(goalY - ry, goalX - rx) + PI;
            odomDesiredHeadingDeg = odomDesiredHeadingRad * RAD_TO_DEG;

            lastRobotX = rx;
            lastRobotY = ry;
            aimDirty   = false;
        }
    }

    private static double wrapPiFast(double angle) {
        if      (angle >  PI) angle -= TWO_PI;
        else if (angle < -PI) angle += TWO_PI;
        return angle;
    }

    public double getTurnPower() { return turnPower; }

    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    public double getShooterTargetX() { return shooterTargetX; }
    public double getShooterTargetY() { return shooterTargetY; }

    public boolean isUsingFarGoal() { return usingFarGoal; }

    public double getOdomDesiredHeadingRad() { return odomDesiredHeadingRad; }
    public double getOdomDesiredHeadingDeg() { return odomDesiredHeadingDeg; }
    public double getOdomErrorDegForGate() { return odomErrorDegForGate; }

    public boolean isUsingFastProfile() { return usingFastProfile; }

    public double getLastErrorDegForTelemetry() { return lastErrorDegForTelemetry; }

    public void telemetry() {
        if (telemetry == null) return;

        telemetry.addData("Aim Turn",        turnPower);
        telemetry.addData("Aim Error Deg",   lastErrorDegForTelemetry);
        telemetry.addData("Aim Desired Deg", odomDesiredHeadingDeg);
        telemetry.addData("Aim Goal X",      goalX);
        telemetry.addData("Aim Goal Y",      goalY);
        telemetry.addData("Aim Zone",        usingFarGoal ? "Far" : "Close");
        telemetry.addData("Aim Profile",     usingFastProfile ? "Fast" : "Precise");

        telemetry.addData("Shooter Target X", shooterTargetX);
        telemetry.addData("Shooter Target Y", shooterTargetY);
    }
}