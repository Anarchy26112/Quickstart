package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HamiltonParams {

    // --- Teleop drive ---
    public static double NORMAL_SPEED = 0.40;
    public static double MAX_AUTO_TURN = 0.68;

    // --- Intake heading assist ---
    public static double HEADING_kP = 0.2;

    // --- Goal position for odom auto aim (BLUE) ---
    public static double GOAL_X = 56.0;
    public static double GOAL_Y_BLUE = -132.0;
    public static double GOAL_Y_RED = 132.0;

    // --- Y-based aim heading offsets ---
    public static double Y_AIM_OFFSET_FAR_DEG = 0.0;
    public static double Y_AIM_OFFSET_MID_DEG = 0.0;
    public static double Y_AIM_OFFSET_NEAR_DEG = 2.4;

    // --- Profile switch threshold ---
    public static double FAST_AIM_Y_THRESHOLD = -36.0;

    // --- Fast / aggressive profile (robotY < -36) ---
    public static double FAST_KP_TURN = 0.0102;
    public static double FAST_KD_TURN = 0.00165;
    public static double FAST_kS_VOLTAGE_COMP = 0.04;
    public static double FAST_ERROR_DEADBAND_DEG = 1.5;

    // --- Precise / passive profile ---
    public static double PRECISE_KP_TURN = 0.0102;
    public static double PRECISE_KD_TURN = 0.00165;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.047;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.4;

    // hardware names unchanged
    public static double SHOOTER_MAX_VELOCITY = 2797.2;

    // --- Shoot on the Move ---
    public static boolean SHOOT_ON_THE_MOVE_ENABLED = true;
    public static double PROJECTILE_SPEED_INCHES_PER_SEC = 100;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_GATE = "gateServo";

    // ========== GATE ==========
    public static final double GATE_BLOCK_POS = 0.20;
    public static final double GATE_OPEN_POS = 0.67;
}