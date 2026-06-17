package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HamiltonParams {

    private HamiltonParams() {}

    // ========== TELEOP DRIVE ==========
    public static double NORMAL_SPEED = 0.30;
    public static double MAX_AUTO_TURN = 0.68;

    // ========== INTAKE HEADING ASSIST ==========
    public static double HEADING_kP = 0.2;

    // ========== GOAL POSITION FOR ODOM AUTO AIM ==========

    // Close-zone goal.
    // Used when robotY >= AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the FAST / aggressive turning profile.
    public static double GOAL_CLOSE_X_BLUE = 7;
    public static double GOAL_CLOSE_Y_BLUE = 134.5;

    // Far-zone goal.
    // Used when robotY < AIM_FAR_ZONE_Y_THRESHOLD.
    // This zone uses the PRECISE / passive turning profile.
    public static double GOAL_FAR_X_BLUE = 7;
    public static double GOAL_FAR_Y_BLUE = 134.5;

    // ========== SHOOTER DISTANCE LOOKUP TARGET ==========
    // This is separate from the aim goal.
    // Blue uses (0, 141.5), red is mirrored by FieldMirror.
    public static double SHOOTER_TARGET_X_BLUE = 0.0;
    public static double SHOOTER_TARGET_Y_BLUE = 141.5;

    // ========== ZONE SWITCH THRESHOLD ==========
    // robotY < 36  -> Far zone   -> precise profile
    // robotY >= 36 -> Close zone -> fast profile
    public static double AIM_FAR_ZONE_Y_THRESHOLD = 40.0;

    // Kept only for dashboard compatibility / tuning reference.
    // In the corrected GoalAimController, profile selection should be:
    //
    //     final boolean isFar = currentRobotY < AIM_FAR_ZONE_Y_THRESHOLD;
    //     final boolean useFast = !isFar;
    //
    // So this threshold should stay equal to AIM_FAR_ZONE_Y_THRESHOLD.
    public static double FAST_AIM_Y_THRESHOLD = 36.0;

    // ========== CLOSE ZONE: FAST / AGGRESSIVE PROFILE ==========
    // Used when robotY >= 36.
    //
    //
    // This should still turn quickly, but with less snap / derivative kick.
    public static double FAST_KP_TURN = 0.0095;
    public static double FAST_KD_TURN = 0.0012;
    public static double FAST_kS_VOLTAGE_COMP = 0.059;
    public static double FAST_ERROR_DEADBAND_DEG = 1.0;

    // ========== FAR ZONE: PRECISE / PASSIVE PROFILE ==========
    // Used when robotY < 36.
    //
    // Softer than the original far-zone behavior.
    // Better for reducing oscillation when aiming from farther away.
    public static double PRECISE_KP_TURN = 0.0065;
    public static double PRECISE_KD_TURN = 0.0010;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.067;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.67;

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