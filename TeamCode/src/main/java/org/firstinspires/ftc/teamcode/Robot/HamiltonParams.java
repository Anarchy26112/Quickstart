package org.firstinspires.ftc.teamcode.Robot;
public class HamiltonParams {
    //Sir Hamiltonian the Clanker LXVII

    // ========== DRIVETRAIN ==========
    public static double NORMAL_SPEED = 0.55;
    public static double FULL_SPEED = 1.0;

    // ========== INTAKE ==========
    public static double INTAKE_POWER = 0.8;
    public static double SPIT_POWER = -0.5;

    // ========== SHOOTER ==========

    public static double SHOOTER_MAX_VELOCITY = 2797.2;

    // Velocity presets (ticks per second)
    public static double HIGH_VELOCITY_THRESHOLD = 2240.0; // 80% of max
    public static double LOW_VELOCITY_THRESHOLD = 1400.0;  // 50% of max

    // ========== PUSHER ==========
    public static double PUSHER_RETRACTED_POS = 0.31;
    public static double PUSHER_EXTENDED_POS = 0.26;
    public static double PUSHER_PUSH_DURATION_MS = 200;
    public static double PUSHER_RETRACT_DELAY_MS = 100;

    // ========== SPINDEX ==========

    public static final long INTAKE_SERVO_TRAVEL_TIME_MS = 350;
    public static final long OUTTAKE_SERVO_TRAVEL_TIME_MS = 250;
    public static final long MOVE_DELAY_MS = 10;
    public static double [] OFFSETS = new double [] {0.032, 0.032, 0.032, 0.032, 0.032, 0.032};
    // 0032
    // ========== COLOR SENSOR ==========
    // Distance threshold for reliable color detection (in mm)
    public static final double MAX_DETECTION_DISTANCE_MM = 50.0;
    public static final float MIN_BRIGHTNESS = 0.1f;

    // ========== LIMELIGHT PID CONTROL ===========
    // PID constants for turn control
    public static double Kp_TURN = 0.02;  // Proportional gain
    public static double Ki_TURN =  0.0;     // Integral gain e.g 0.0001 to 0.001        0.0005
    public static double Kd_TURN = 0.0;     // Derivative gain e.g 0.1 to 0.5       0.2

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