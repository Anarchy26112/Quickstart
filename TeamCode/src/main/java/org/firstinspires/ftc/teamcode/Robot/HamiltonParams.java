package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HamiltonParams {

    // --- Teleop drive ---
    public static double NORMAL_SPEED = 0.007;
    public static double MAX_AUTO_TURN = 0.80;

    // --- Intake heading assist ---
    public static double HEADING_kP = 1.2;

    // --- Goal position for odom auto aim (BLUE) ---
    public static double GOAL_X = 56.0;
    public static double GOAL_Y = -132.0;

    // --- Y-based aim heading offsets ---
    public static double Y_AIM_OFFSET_FAR_DEG = 13.0;   // CLOSE ZONE
    public static double Y_AIM_OFFSET_MID_DEG = 9.0;   // MID ZONE
    public static double Y_AIM_OFFSET_NEAR_DEG = 2.8;   // FAR ZONE

    // --- Profile switch threshold ---
    public static double FAST_AIM_Y_THRESHOLD = -36.0;

    // --- Fast / aggressive profile (robotY < -36) ---
    public static double FAST_KP_TURN = 0.012;
    public static double FAST_KD_TURN = 0.0010;
    public static double FAST_kS_VOLTAGE_COMP = 0.026;
    public static double FAST_ERROR_DEADBAND_DEG = 0.7;

    // --- Precise / passive profile (robotY > -36) ---
    public static double PRECISE_KP_TURN = 0.01;
    public static double PRECISE_KD_TURN = 0.0015;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.023;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.35;

    // --- Final aim checks ---
    public static double ODOM_AIM_DEADBAND_DEG = 1.0;

    // --- Limelight trim ---
    public static double LIMELIGHT_MOUNT_OFFSET_DEG = 0.7;
    public static double LIMELIGHT_TRIM_kP = 0.020;
    public static double LIMELIGHT_TRIM_MAX = 0.30;
    public static double LIMELIGHT_TX_DEADBAND_DEG = 0.30;
    public static double VISION_ENABLE_ODOM_ERROR_DEG = 5.0;

    // --- Shoot ready gating (profile-based) ---
    public static double FAST_SHOOT_READY_HEADING_ERROR_DEG = 4.0;
    public static double PRECISE_SHOOT_READY_HEADING_ERROR_DEG = 1.35;
    public static double FAST_SHOOT_READY_MAX_TURN = 0.10;
    public static double PRECISE_SHOOT_READY_MAX_TURN = 0.035;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_LIMELIGHT = "Limelight";
    public static final String HW_GATE = "gateServo";

    // ========== GATE ==========
    public static final double GATE_BLOCK_POS = 0.20;
    public static final double GATE_OPEN_POS = 0.67;

    public static double SHOOTER_MAX_VELOCITY = 2797.2;
}