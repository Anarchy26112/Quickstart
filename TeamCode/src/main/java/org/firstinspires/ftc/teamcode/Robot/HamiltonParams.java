package org.firstinspires.ftc.teamcode.Robot;

public class HamiltonParams {
    //Sir Hamiltonian the Clanker LXVII

    // ========== DRIVETRAIN ==========
    public static double NORMAL_SPEED = 0.3;
    public static double FULL_SPEED = 1.0;

    // ========== INTAKE ==========
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double SPIT_POWER = 0.5;

    // ========== GATE ===========
    public static final double GATE_BLOCK_POS = 0.687; // 0.687
    public static final double GATE_OPEN_POS = 0.39; // adjust

    // ========== SHOOTER ==========
    public static double SHOOTER_MAX_VELOCITY = 2797.2;

    // Velocity presets (ticks per second)
    public static double HIGH_VELOCITY_THRESHOLD = 2255.0; // 80% of max
    public static double LOW_VELOCITY_THRESHOLD = 1800.0;  // 50% of max

    // ========== COLOR SENSOR ==========
    // Distance threshold for reliable color detection (in mm)
    public static final double MAX_DETECTION_DISTANCE_MM = 50.0; // Tune so that we find the exact distance in millimeters
    public static final float MIN_BRIGHTNESS = 0.1f;

    // ========== LIMELIGHT PID CONTROL ===========
    // PID constants for turn control
    public static double Kp_TURN = 0.012; // 0.0075
    public static double Ki_TURN = 0.0;   // 0.016
    public static double Kd_TURN = 0.0012; // 0.0018
    public static double MIN_TURN_POWER = 0.087;;  // 0.084
    public static double HEADING_kP = 0.4;
    public static final double MAX_AUTO_TURN = 0.37;
    // ========== LIMELIGHT AUTO-OFFSET ==========
    public static double OFFSET_SWITCH_DISTANCE_IN = 90.0; // tune this (inches). Remember it's for far zone.
    public static double TX_OFFSET_FAR_DEG_RED = -1.0;
    public static double TX_OFFSET_FAR_DEG_BLUE = 1.0;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_LIMELIGHT = "Limelight";
    public static final String HW_GATE = "gateServo";
}
