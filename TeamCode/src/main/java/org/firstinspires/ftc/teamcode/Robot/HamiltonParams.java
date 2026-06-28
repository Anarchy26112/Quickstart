package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HamiltonParams {

    private HamiltonParams() {}

    // ========== TELEOP DRIVE ==========
    public static double NORMAL_SPEED = 0.30;
    public static double MAX_AUTO_TURN = 0.55;

    // ========== INTAKE HEADING ASSIST ==========
    public static double HEADING_kP = 0.2;

    // ========== GOAL POSITION FOR ODOM AUTO AIM ==========

    // Close-zone goal.
    // Used when robotY >= AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the FAST / aggressive turning profile.
    public static double GOAL_CLOSE_X_BLUE = 5.0;
    public static double GOAL_CLOSE_Y_BLUE = 136.5;

    // Far-zone goal.
    // Used when robotY < AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the PRECISE / passive turning profile.
    //
    // Currently same as close goal.
    // That means the zone switch changes control behavior only, not aim point.
    public static double GOAL_FAR_X_BLUE = 5.0;
    public static double GOAL_FAR_Y_BLUE = 136.5;

    // ========== SHOOTER DISTANCE LOOKUP TARGET ==========
    // This is separate from the aim goal.
    // Blue uses (0, 141.5), red is mirrored by FieldMirror.
    public static double SHOOTER_TARGET_X_BLUE = 0.0;
    public static double SHOOTER_TARGET_Y_BLUE = 141.5;

    // ========== ZONE SWITCH THRESHOLD ==========
    // robotY < 40  -> Far zone   -> precise profile
    // robotY >= 40 -> Close zone -> fast profile
    public static double AIM_FAR_ZONE_Y_THRESHOLD = 47.2;

    // Kept only for dashboard compatibility / tuning reference.
    // In GoalAimController, profile selection is:
    //
    //     final boolean isFar = currentRobotY < AIM_FAR_ZONE_Y_THRESHOLD;
    //     final boolean useFast = !isFar;
    //
    // So this threshold should stay equal to AIM_FAR_ZONE_Y_THRESHOLD.
    public static double FAST_AIM_Y_THRESHOLD = 40.0;

    // ========== CLOSE ZONE: FAST / AGGRESSIVE PROFILE ==========
    public static double FAST_KP_TURN = 0.012;
    public static double FAST_KD_TURN = 0.0020;
    public static double FAST_kS_VOLTAGE_COMP = 0.05;
    public static double FAST_ERROR_DEADBAND_DEG = 2.5;

    // ========== FAR ZONE: PRECISE / PASSIVE PROFILE ==========
    public static double PRECISE_KP_TURN = 0.0070;
    public static double PRECISE_KD_TURN = 0.0023;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.03;;
    public static double PRECISE_ERROR_DEADBAND_DEG = 1.47;

    public static double AIM_SETTLE_VEL_DEG_PER_SEC = 12.0;
    public static double AIM_KS_RAMP_DEADBAND_MULT = 3.0;

    public static double AIM_TARGET_HEADING_VEL_FF = 0.0025;
    public static double AIM_TARGET_HEADING_VEL_MAX_DEG_PER_SEC = 180.0;

    // ========== VOLTAGE COMPENSATION ==========
    public static final double NOMINAL_VOLTAGE = 12.9;
    public static final double VOLTAGE_COMP_POWER = 1.04;

    // ========== TIME CONVERSION ==========
    public static final double NANO_TO_SEC = 1.0e-9;
    public static final double NANO_TO_MS = 1.0e-6;

    // ========== SHOOTER COMMAND OPTIMIZATION ==========
    public static double SHOOTER_COMMAND_EPSILON = 1.0;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_GATE = "gateServo";

    // ========== GATE ==========
    public static double GATE_BLOCK_POS = 0.25;
    public static double GATE_OPEN_POS = 0.583;
}