package org.firstinspires.ftc.teamcode.Robot;

public class HamiltonParams {
    //Sir Hamiltonian the Clanker LXVII

    // ========== DRIVETRAIN ==========
    public static double NORMAL_SPEED = 0.3;
    public static double FULL_SPEED = 1.0;

    // ========== INTAKE ==========
    public static double INTAKE_POWER = 1.0;
    public static double SPIT_POWER = -0.5;

    // ========== SHOOTER ==========
    public static double SHOOTER_MAX_VELOCITY = 2797.2;

    // Velocity presets (ticks per second)
    public static double HIGH_VELOCITY_THRESHOLD = 2255.0; // 80% of max
    public static double LOW_VELOCITY_THRESHOLD = 1400.0;  // 50% of max

    // ========== PUSHER ==========
    public static double PUSHER_RETRACTED_POS = 0.305;
    public static double PUSHER_EXTENDED_POS = 0.265;
    public static double PUSHER_PUSH_DURATION_MS = 175;
    public static double PUSHER_RETRACT_DELAY_MS = 100;

    // ========== SPINDEX ==========
    public static final long INTAKE_SERVO_TRAVEL_TIME_MS = 250;
    public  static final long OUTTAKE_SERVO_TRAVEL_TIME_MS = 333;
    public static final long MOVE_DELAY_MS = 0;
    public static double DEFAULT_KP = 0.006;
    public static double DEFAULT_KD = 0.00048;
    // 312 RPM
    // KP = 0.005
    // KD = 0.00022
    public static double [] OFFSETS = new double [] {0.032, 0.030, 0.032, 0.032, 0.032, 0.034, 0.032, 0.034, 0.032, 0.034, 0.032, 0.034, 0.032, 0.034, 0.032, 0.034, 0.032, 0.034 };
    // 0032
    // ========== COLOR SENSOR ==========
    // Distance threshold for reliable color detection (in mm)
    public static final double MAX_DETECTION_DISTANCE_MM = 30.0; // Tune so that we find the exact distance in millimeters
    public static final float MIN_BRIGHTNESS = 0.1f;

    // ========== LIMELIGHT PID CONTROL ===========
    // PID constants for turn control
    public static double Kp_TURN = 0.012; // 0.0075
    public static double Ki_TURN = 0.0;   // 0.016
    public static double Kd_TURN = 0.0012; // 0.0018
    public static double MIN_TURN_POWER = 0.087;;  // 0.084
    public static double HEADING_kP = 0.4;
    public static final double MAX_AUTO_TURN = 0.33;

    // ========== LIMELIGHT AUTO-OFFSET ==========
    public static double OFFSET_SWITCH_DISTANCE_IN = 90.0; // tune this (inches). Remember it's for far zone.
    public static double TX_OFFSET_FAR_DEG_RED = -1.15;
    public static double TX_OFFSET_FAR_DEG_BLUE = 1.15;


    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_PUSHER = "pusher";
    public static final String HW_SPINDEX = "spinDex";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_LIMELIGHT = "Limelight";
}
