package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class GoalAimController {

    public enum AllianceColor {
        BLUE,
        RED
    }

    private final Telemetry telemetry;

    /*
     * Kept for constructor compatibility.
     * This controller does not directly need follower access if TeleOp passes
     * pose and angular velocity into updateActive().
     */
    @SuppressWarnings("unused")
    private final Follower follower;

    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    private double goalX = 0.0;
    private double goalY = 0.0;

    private double closeGoalX = 0.0;
    private double closeGoalY = 0.0;

    private double farGoalX = 0.0;
    private double farGoalY = 0.0;

    private double shooterTargetX = 0.0;
    private double shooterTargetY = 141.5;

    // Kept for compatibility. Without shoot-on-the-move, this is always the same as goalX / goalY.
    private double effectiveGoalX = 0.0;
    private double effectiveGoalY = 0.0;

    private AllianceColor allianceColor = AllianceColor.BLUE;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;

    private boolean usingFastProfile = false;
    private boolean usingFarGoal = false;
    private boolean goalInitialized = false;
    private boolean lastUsingFarGoal = false;

    // ========== SETTLED STATE TRACKING ==========
    private long settledStartTimeNs = 0L;
    private boolean isSettled = false;

    private static final long SETTLED_THRESHOLD_NS = 50_000_000L; // 50 ms

    private long lastUpdateNs = 0L;

    private double odomDesiredHeadingRad = 0.0;
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate = 0.0;
    private double lastErrorDegForTelemetry = 0.0;

    private double aimAngularVelocityDegPerSec = 0.0;

    // For telemetry / tuning.
    private double lastKsScale = 0.0;
    private double lastAimVoltageComp = 1.0;

    private static final double PI = Math.PI;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private static final double MIN_AIM_DT = 0.001;
    private static final double MAX_AIM_DT = 0.1;

    private double cachedKP = PRECISE_KP_TURN;
    private double cachedKD = PRECISE_KD_TURN;
    private double cachedKS = PRECISE_kS_VOLTAGE_COMP;
    private double cachedDeadband = PRECISE_ERROR_DEADBAND_DEG;

    private boolean cachedIsFast = false;
    private boolean cachedProfileInitialized = false;

    // ========== DERIVATIVE / VELOCITY TRACKING ==========

    // Prevents first-frame derivative spike after forceIdle(), reset(), or profile switch.
    private boolean derivativeInitialized = false;

    // Robot yaw velocity used for damping and settling.
    // Positive should match Pedro heading-positive direction.
    private double robotYawVelocityDegPerSec = 0.0;

    // Fallback heading velocity from odometry heading if Pedro angular velocity is unavailable.
    private boolean headingVelocityInitialized = false;
    private double lastRobotHeadingForVelocityRad = 0.0;

    // Target heading velocity caused by robot translation.
    private boolean targetVelocityInitialized = false;
    private double lastDesiredHeadingForTargetVelocityRad = 0.0;
    private double targetHeadingVelocityDegPerSec = 0.0;

    // Raw error derivative kept only for telemetry/debugging.
    private double rawErrorDerivativeDegPerSec = 0.0;

    public GoalAimController(Telemetry telemetry) {
        this(null, telemetry);
    }

    public GoalAimController(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        setAlliance(AllianceColor.BLUE);
    }

    public void setRobotPose(double x, double y, double headingRad) {
        robotX = x;
        robotY = y;
        robotHeadingRad = headingRad;
    }

    /*
     * Kept only for compatibility with older TeleOp code.
     * Shoot-on-the-move has been removed, so translational velocity is no longer used.
     */
    public void setRobotVelocity(double vx, double vy) {
        // No-op.
    }

    public void setAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        final Alliance mirrorAlliance =
                allianceColor == AllianceColor.BLUE ? Alliance.BLUE : Alliance.RED;

        final Pose closeGoalPose =
                FieldMirror.pose(mirrorAlliance, GOAL_CLOSE_X_BLUE, GOAL_CLOSE_Y_BLUE, 0.0);

        final Pose farGoalPose =
                FieldMirror.pose(mirrorAlliance, GOAL_FAR_X_BLUE, GOAL_FAR_Y_BLUE, 0.0);

        final Pose shooterTargetPose =
                FieldMirror.pose(mirrorAlliance, SHOOTER_TARGET_X_BLUE, SHOOTER_TARGET_Y_BLUE, 0.0);

        closeGoalX = closeGoalPose.getX();
        closeGoalY = closeGoalPose.getY();

        farGoalX = farGoalPose.getX();
        farGoalY = farGoalPose.getY();

        shooterTargetX = shooterTargetPose.getX();
        shooterTargetY = shooterTargetPose.getY();

        goalInitialized = false;

        updateGoalForCurrentZone(robotY < AIM_FAR_ZONE_Y_THRESHOLD);
    }

    private void updateGoalForCurrentZone(boolean isFar) {
        usingFarGoal = isFar;
        lastUsingFarGoal = isFar;
        goalInitialized = true;

        goalX = isFar ? farGoalX : closeGoalX;
        goalY = isFar ? farGoalY : closeGoalY;

        effectiveGoalX = goalX;
        effectiveGoalY = goalY;
    }

    public void reset() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeadingRad = 0.0;

        lastHeadingErrorDeg = 0.0;
        lastErrorDegForTelemetry = 0.0;
        aimAngularVelocityDegPerSec = 0.0;

        turnPower = 0.0;
        lastKsScale = 0.0;
        lastAimVoltageComp = 1.0;

        usingFastProfile = false;
        usingFarGoal = false;
        lastUsingFarGoal = false;
        goalInitialized = false;

        cachedProfileInitialized = false;
        cachedIsFast = false;

        cachedKP = PRECISE_KP_TURN;
        cachedKD = PRECISE_KD_TURN;
        cachedKS = PRECISE_kS_VOLTAGE_COMP;
        cachedDeadband = PRECISE_ERROR_DEADBAND_DEG;

        isSettled = false;
        settledStartTimeNs = 0L;

        lastUpdateNs = 0L;

        derivativeInitialized = false;

        robotYawVelocityDegPerSec = 0.0;

        headingVelocityInitialized = false;
        lastRobotHeadingForVelocityRad = 0.0;

        targetVelocityInitialized = false;
        lastDesiredHeadingForTargetVelocityRad = 0.0;
        targetHeadingVelocityDegPerSec = 0.0;

        rawErrorDerivativeDegPerSec = 0.0;

        setAlliance(allianceColor);
    }

    public void forceIdle(long nowNs) {
        turnPower = 0.0;
        lastKsScale = 0.0;
        lastAimVoltageComp = 1.0;

        /*
         * Do not pretend the previous heading error was zero.
         * That creates a derivative spike when aim starts again.
         *
         * Instead, force the derivative state to re-initialize cleanly
         * on the next active update.
         */
        derivativeInitialized = false;

        lastHeadingErrorDeg = 0.0;
        aimAngularVelocityDegPerSec = 0.0;
        robotYawVelocityDegPerSec = 0.0;
        rawErrorDerivativeDegPerSec = 0.0;

        headingVelocityInitialized = false;

        targetVelocityInitialized = false;
        targetHeadingVelocityDegPerSec = 0.0;

        lastUpdateNs = nowNs;

        isSettled = false;
        settledStartTimeNs = 0L;
    }

    /*
     * Compatibility version.
     */
    public void updateActive(
            double currentRobotX,
            double currentRobotY,
            double currentRobotHeading,
            long nowNs,
            double dt
    ) {
        updateActive(
                currentRobotX,
                currentRobotY,
                currentRobotHeading,
                nowNs,
                dt,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                1.0
        );
    }

    /*
     * PedroPathing version without voltage compensation.
     */
    public void updateActive(
            double currentRobotX,
            double currentRobotY,
            double currentRobotHeading,
            long nowNs,
            double dt,
            double followerVx,
            double followerVy,
            double followerHeadingVelocityRadPerSec
    ) {
        updateActive(
                currentRobotX,
                currentRobotY,
                currentRobotHeading,
                nowNs,
                dt,
                followerVx,
                followerVy,
                followerHeadingVelocityRadPerSec,
                1.0
        );
    }

    /*
     * Preferred version.
     *
     * followerVx and followerVy are kept only for compatibility.
     * Shoot-on-the-move has been removed, so translational velocity is not used.
     *
     * followerHeadingVelocityRadPerSec:
     *     Robot yaw/angular velocity, radians/sec.
     *     Use follower.getAngularVelocity().
     *
     * aimVoltageComp:
     *     Usually NOMINAL_VOLTAGE / currentVoltage.
     */
    public void updateActive(
            double currentRobotX,
            double currentRobotY,
            double currentRobotHeading,
            long nowNs,
            double dt,
            double followerVx,
            double followerVy,
            double followerHeadingVelocityRadPerSec,
            double aimVoltageComp
    ) {
        robotX = currentRobotX;
        robotY = currentRobotY;
        robotHeadingRad = currentRobotHeading;
        lastUpdateNs = nowNs;

        if (dt < MIN_AIM_DT) {
            dt = MIN_AIM_DT;
        } else if (dt > MAX_AIM_DT) {
            dt = MAX_AIM_DT;
        }

        // ========== AIM VOLTAGE COMPENSATION ==========
        if (!Double.isFinite(aimVoltageComp) || aimVoltageComp <= 0.0) {
            aimVoltageComp = 1.0;
        }

        if (aimVoltageComp < 0.85) {
            aimVoltageComp = 0.85;
        } else if (aimVoltageComp > 1.45) {
            aimVoltageComp = 1.45;
        }

        lastAimVoltageComp = aimVoltageComp;

        // ========== ZONE / PROFILE SELECTION ==========

        final boolean isFar = currentRobotY < AIM_FAR_ZONE_Y_THRESHOLD;

        boolean resetDerivative = false;

        if (isFar != lastUsingFarGoal || !goalInitialized) {
            updateGoalForCurrentZone(isFar);
            resetDerivative = true;
        }

        updateOdomAim(currentRobotX, currentRobotY);

        final double errorRad =
                wrapPi(odomDesiredHeadingRad - currentRobotHeading);

        final double errorDeg =
                errorRad * RAD_TO_DEG;

        lastErrorDegForTelemetry = errorDeg;
        odomErrorDegForGate = errorDeg;

        final boolean useFast = !isFar;

        if (!cachedProfileInitialized || useFast != cachedIsFast) {
            cachedProfileInitialized = true;
            cachedIsFast = useFast;

            cachedKP = useFast ? FAST_KP_TURN : PRECISE_KP_TURN;
            cachedKD = useFast ? FAST_KD_TURN : PRECISE_KD_TURN;
            cachedKS = useFast ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
            cachedDeadband = useFast ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

            usingFastProfile = useFast;

            resetDerivative = true;
        }

        if (resetDerivative) {
            derivativeInitialized = false;
            headingVelocityInitialized = false;
            targetVelocityInitialized = false;

            rawErrorDerivativeDegPerSec = 0.0;
            robotYawVelocityDegPerSec = 0.0;
            aimAngularVelocityDegPerSec = 0.0;
            targetHeadingVelocityDegPerSec = 0.0;

            settledStartTimeNs = 0L;
            isSettled = false;
        }

        // ========== ERROR DERIVATIVE TELEMETRY ONLY ==========

        if (!derivativeInitialized) {
            derivativeInitialized = true;

            lastHeadingErrorDeg = errorDeg;
            rawErrorDerivativeDegPerSec = 0.0;

            settledStartTimeNs = 0L;
            isSettled = false;
        } else {
            rawErrorDerivativeDegPerSec =
                    (errorDeg - lastHeadingErrorDeg) / dt;

            lastHeadingErrorDeg = errorDeg;
        }

        // ========== ROBOT YAW VELOCITY FOR KD DAMPING ==========

        if (Double.isFinite(followerHeadingVelocityRadPerSec)) {
            robotYawVelocityDegPerSec =
                    followerHeadingVelocityRadPerSec * RAD_TO_DEG;
        } else {
            if (!headingVelocityInitialized) {
                headingVelocityInitialized = true;
                lastRobotHeadingForVelocityRad = currentRobotHeading;
                robotYawVelocityDegPerSec = 0.0;
            } else {
                final double headingDeltaRad =
                        wrapPi(currentRobotHeading - lastRobotHeadingForVelocityRad);

                robotYawVelocityDegPerSec =
                        headingDeltaRad * RAD_TO_DEG / dt;

                lastRobotHeadingForVelocityRad = currentRobotHeading;
            }
        }

        aimAngularVelocityDegPerSec = robotYawVelocityDegPerSec;

        // ========== TARGET HEADING VELOCITY FEEDFORWARD ==========
        //
        // This is based on desired heading change as the robot moves around the field.
        // It helps the robot avoid lagging behind the moving aim angle.

        if (!targetVelocityInitialized) {
            targetVelocityInitialized = true;
            lastDesiredHeadingForTargetVelocityRad = odomDesiredHeadingRad;
            targetHeadingVelocityDegPerSec = 0.0;
        } else {
            final double desiredHeadingDeltaRad =
                    wrapPi(odomDesiredHeadingRad - lastDesiredHeadingForTargetVelocityRad);

            targetHeadingVelocityDegPerSec =
                    desiredHeadingDeltaRad * RAD_TO_DEG / dt;

            lastDesiredHeadingForTargetVelocityRad = odomDesiredHeadingRad;

            if (targetHeadingVelocityDegPerSec > AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC) {
                targetHeadingVelocityDegPerSec = AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC;
            } else if (targetHeadingVelocityDegPerSec < -AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC) {
                targetHeadingVelocityDegPerSec = -AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC;
            }
        }

        final double absError = Math.abs(errorDeg);
        final double absAngularVelocity = Math.abs(robotYawVelocityDegPerSec);

        // ========== VELOCITY-BASED SETTLING ==========
        //
        // Settled means safe to shoot.
        // It does NOT mean stop controlling heading.

        if (absError <= cachedDeadband &&
                absAngularVelocity <= AIM_SETTLE_VEL_DEG_PER_SEC) {

            if (settledStartTimeNs == 0L) {
                settledStartTimeNs = nowNs;
            }

            if (nowNs - settledStartTimeNs >= SETTLED_THRESHOLD_NS) {
                isSettled = true;
            }

        } else {
            settledStartTimeNs = 0L;
            isSettled = false;
        }

        // ========== PD CONTROL ==========
        //
        // KD uses robot yaw velocity, not raw error derivative.
        // This is more stable during moving aim.

        final double rotationalDampingDegPerSec =
                -robotYawVelocityDegPerSec;

        final double movingTargetFeedforward =
                AIM_TARGET_HEADING_VEL_FF * targetHeadingVelocityDegPerSec;

        double output =
                cachedKP * errorDeg
                        + cachedKD * rotationalDampingDegPerSec
                        + movingTargetFeedforward;

        // ========== RAMPED kS ==========

        lastKsScale = 0.0;

        if (absError > cachedDeadband) {
            final double ksFullError =
                    Math.max(
                            cachedDeadband * AIM_KS_RAMP_DEADBAND_MULT,
                            cachedDeadband + 0.001
                    );

            double ksScale =
                    (absError - cachedDeadband) / (ksFullError - cachedDeadband);

            if (ksScale > 1.0) {
                ksScale = 1.0;
            } else if (ksScale < 0.0) {
                ksScale = 0.0;
            }

            lastKsScale = ksScale;

            output += Math.copySign(cachedKS * ksScale, errorDeg);
        }

        // ========== FINAL VOLTAGE COMPENSATION ==========
        output *= aimVoltageComp;

        turnPower = clamp(output, -MAX_AUTO_TURN, MAX_AUTO_TURN);
    }

    private void updateOdomAim(
            final double rx,
            final double ry
    ) {
        effectiveGoalX = goalX;
        effectiveGoalY = goalY;

        final double dx = goalX - rx;
        final double dy = goalY - ry;

        odomDesiredHeadingRad =
                Math.atan2(dy, dx) + PI;

        odomDesiredHeadingDeg =
                odomDesiredHeadingRad * RAD_TO_DEG;

        final double odomErrorRad =
                wrapPi(odomDesiredHeadingRad - robotHeadingRad);

        odomErrorDegForGate =
                odomErrorRad * RAD_TO_DEG;
    }

    private static double wrapPi(double angle) {
        double wrapped = angle % (2.0 * Math.PI);

        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }

        return wrapped;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTurnPower() {
        return turnPower;
    }

    public boolean isSettled() {
        return isSettled;
    }

    public double getShooterTargetX() {
        return shooterTargetX;
    }

    public double getShooterTargetY() {
        return shooterTargetY;
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

    public double getEffectiveGoalX() {
        return effectiveGoalX;
    }

    public double getEffectiveGoalY() {
        return effectiveGoalY;
    }

    public double getTargetHeadingVelocityDegPerSec() {
        return targetHeadingVelocityDegPerSec;
    }

    public double getRobotYawVelocityDegPerSec() {
        return robotYawVelocityDegPerSec;
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Aim Turn", turnPower);
        telemetry.addData("Aim Settled", isSettled);

        telemetry.addData("Aim Goal", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("Aim Effective Goal", "(%.1f, %.1f)", effectiveGoalX, effectiveGoalY);

        telemetry.addData("Aim Robot Pose", "(%.1f, %.1f, %.1fdeg)",
                robotX,
                robotY,
                robotHeadingRad * RAD_TO_DEG);

        telemetry.addData("Aim Desired Heading Deg", odomDesiredHeadingDeg);
        telemetry.addData("Aim Error Deg", lastErrorDegForTelemetry);

        telemetry.addData("Aim Angular Vel Deg/Sec", aimAngularVelocityDegPerSec);
        telemetry.addData("Aim Robot Yaw Vel Deg/Sec", robotYawVelocityDegPerSec);
        telemetry.addData("Aim Raw Error Deriv Deg/Sec", rawErrorDerivativeDegPerSec);
        telemetry.addData("Aim Target Heading Vel Deg/Sec", targetHeadingVelocityDegPerSec);

        telemetry.addData("Aim kS Scale", lastKsScale);
        telemetry.addData("Aim Voltage Comp", lastAimVoltageComp);

        telemetry.addData("Aim Zone", usingFarGoal ? "Far" : "Close");
        telemetry.addData("Aim Profile", usingFastProfile ? "Fast" : "Precise");
    }
}