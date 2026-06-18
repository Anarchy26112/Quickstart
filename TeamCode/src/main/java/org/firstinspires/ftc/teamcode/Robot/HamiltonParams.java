package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HamiltonParams {

    private HamiltonParams() {}

    // ========== TELEOP DRIVE ==========
    public static double NORMAL_SPEED = 0.30;
    public static double MAX_AUTO_TURN = 0.8;

    // ========== INTAKE HEADING ASSIST ==========
    public static double HEADING_kP = 0.2;

    // ========== GOAL POSITION FOR ODOM AUTO AIM ==========

    // Close-zone goal.
    // Used when robotY >= AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the FAST / aggressive turning profile.
    public static double GOAL_CLOSE_X_BLUE = 9.0;
    public static double GOAL_CLOSE_Y_BLUE = 132.5;

    // Far-zone goal.
    // Used when robotY < AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the PRECISE / passive turning profile.
    //
    // Currently same as close goal.
    // That means the zone switch changes control behavior only, not aim point.
    public static double GOAL_FAR_X_BLUE = 9.0;
    public static double GOAL_FAR_Y_BLUE = 132.5;

    // ========== SHOOTER DISTANCE LOOKUP TARGET ==========
    // This is separate from the aim goal.
    // Blue uses (0, 141.5), red is mirrored by FieldMirror.
    public static double SHOOTER_TARGET_X_BLUE = 0.0;
    public static double SHOOTER_TARGET_Y_BLUE = 141.5;

    // ========== ZONE SWITCH THRESHOLD ==========
    // robotY < 40  -> Far zone   -> precise profile
    // robotY >= 40 -> Close zone -> fast profile
    public static double AIM_FAR_ZONE_Y_THRESHOLD = 40.0;

    // Kept only for dashboard compatibility / tuning reference.
    // In GoalAimController, profile selection is:
    //
    //     final boolean isFar = currentRobotY < AIM_FAR_ZONE_Y_THRESHOLD;
    //     final boolean useFast = !isFar;
    //
    // So this threshold should stay equal to AIM_FAR_ZONE_Y_THRESHOLD.
    public static double FAST_AIM_Y_THRESHOLD = 40.0;

    // ========== CLOSE ZONE: FAST / AGGRESSIVE PROFILE ==========
    // Used when robotY >= AIM_FAR_ZONE_Y_THRESHOLD.
    //
    // Safer starting value than 0.0095. Increase only if the robot is too slow.
    public static double FAST_KP_TURN = 0.015;
    public static double FAST_KD_TURN = 0.0016;
    public static double FAST_kS_VOLTAGE_COMP = 0.064;
    public static double FAST_ERROR_DEADBAND_DEG = 3.5;

    // ========== FAR ZONE: PRECISE / PASSIVE PROFILE ==========
    // Used when robotY < AIM_FAR_ZONE_Y_THRESHOLD.
    public static double PRECISE_KP_TURN = 0.007;
    public static double PRECISE_KD_TURN = 0.0014;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.064;

    public static double PRECISE_ERROR_DEADBAND_DEG = 1.6;

    // ========== AIM SETTLING / kS RAMP ==========

    // Robot must be under this yaw velocity and within deadband before "settled."
    public static double AIM_SETTLE_VEL_DEG_PER_SEC = 25.0;

    // Full kS begins at deadband * this multiplier.
    //
    // Example:
    // deadband = 1.0 deg, multiplier = 3.0
    // kS is:
    //     0% at 1.0 deg
    //     50% at 2.0 deg
    //     100% at 3.0 deg
    public static double AIM_KS_RAMP_DEADBAND_MULT = 3.0;

    // ========== MOVING TARGET TRACKING ==========
    /*
     * Feedforward for target heading movement caused by robot translation.
     *
     * Units:
     *     turn power per deg/sec of target-heading motion
     *
     * Start small.
     * If the robot lags while strafing past the goal, increase slightly.
     * If it over-leads / feels twitchy while moving, decrease.
     */
    public static double AIM_TARGET_HEADING_VEL_FF = 0.0018;

    // Clamp target heading velocity used for feedforward.
    // Prevents a big spike when very close to the target point or from odom noise.
    public static double AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC = 120.0;

    // ========== VOLTAGE COMPENSATION ==========
    public static final double NOMINAL_VOLTAGE = 12.5;
    public static final double VOLTAGE_COMP_POWER = 1.0;

    // ========== TIME CONVERSION ==========
    public static final double NANO_TO_SEC = 1.0e-9;
    public static final double NANO_TO_MS = 1.0e-6;

    // ========== SHOOTER COMMAND OPTIMIZATION ==========
    public static final double SHOOTER_COMMAND_EPSILON = 1.0;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_GATE = "gateServo";

    // ========== GATE ==========
    public static final double GATE_BLOCK_POS = 0.26;
    public static final double GATE_OPEN_POS = 0.51;
}