package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HamiltonParams {

    private HamiltonParams() {}

    // --- Teleop drive ---
    public static double NORMAL_SPEED = 0.30;
    public static double MAX_AUTO_TURN = 0.68;

    // --- Intake heading assist ---
    public static double HEADING_kP = 0.2;

    // --- Goal position for odom auto aim ---
    public static double GOAL_CLOSE_X_BLUE = 10.0;
    public static double GOAL_CLOSE_Y_BLUE = 131.5;

    // BLUE far zone goal: robotY < 36
    public static double GOAL_FAR_X_BLUE = 2.0;
    public static double GOAL_FAR_Y_BLUE = 141.5;

    // --- Shooter distance lookup point ---
    // This is separate from the aim goal.
    // Blue uses (0, 141.5), red is mirrored by FieldMirror.
    public static double SHOOTER_TARGET_X_BLUE = 0.0;
    public static double SHOOTER_TARGET_Y_BLUE = 141.5;

    // --- Zone switch threshold ---
    public static double AIM_FAR_ZONE_Y_THRESHOLD = 36.0;

    // --- Profile switch threshold ---
    public static double FAST_AIM_Y_THRESHOLD = 36.0;

    // --- Fast / aggressive profile ---
    public static double FAST_KP_TURN = 0.0102;
    public static double FAST_KD_TURN = 0.00165;
    public static double FAST_kS_VOLTAGE_COMP = 0.047;
    public static double FAST_ERROR_DEADBAND_DEG = 1.5;

    // --- Precise / passive profile ---
    public static double PRECISE_KP_TURN = 0.0075;
    public static double PRECISE_KD_TURN = 0.0015;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.05;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.5;

    // --- Voltage compensation ---
    public static final double NOMINAL_VOLTAGE = 12.2;
    public static final double VOLTAGE_COMP_POWER = 1.0;

    // --- Time conversion ---
    public static final double NANO_TO_SEC = 1.0e-9;
    public static final double NANO_TO_MS = 1.0e-6;

    // --- Shooter command optimization ---
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