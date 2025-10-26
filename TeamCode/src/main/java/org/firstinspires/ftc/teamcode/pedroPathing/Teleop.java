package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp")
public class Teleop extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Intake intake;
    private SpinDex spin_dex;
    private Shooter shooter;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private boolean intakeActive = false;
    private boolean lastAPressed = false; // debounce helper
    private Pose startingPose = null;

    @Override
    public void init() {
        // Initialize motors directly
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions for mecanum drive
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize other subsystems
        intake = new Intake(hardwareMap);
        spin_dex = new SpinDex(hardwareMap);
        shooter = new Shooter(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug("Status: Initialized");
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double axial = -gamepad1.left_stick_y;  // forward/backward
        double lateral = gamepad1.left_stick_x; // strafe
        double yaw = gamepad1.right_stick_x;    // turn

        // Apply speed scaling
        double speedMultiplier = gamepad1.right_bumper ? 1.0 : 0.55;
        axial *= speedMultiplier;
        lateral *= speedMultiplier;
        yaw *= speedMultiplier;

        // Calculate motor powers
        double lf = axial + lateral + yaw;
        double lb = axial - lateral + yaw;
        double rf = axial - lateral - yaw;
        double rb = axial + lateral - yaw;

        // Normalize powers so none exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb)));

        if (maxPower > 1.0) {
            lf /= maxPower;
            lb /= maxPower;
            rf /= maxPower;
            rb /= maxPower;
        }

        // Set motor powers
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);

        // Log drive info
        if (gamepad1.right_bumper) {
            telemetryM.debug("Drive Mode: Full Speed");
        } else {
            telemetryM.debug("Drive Mode: Reduced Speed (55%)");
        }

        // intake toggle with proper debouncing
        if (gamepad2.a && !lastAPressed) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intake();
            } else {
                intake.resetIntake();
            }
        }
        lastAPressed = gamepad2.a;

        // Add telemetry data
        telemetryM.debug("Intake Active: " + intakeActive);
        telemetryM.debug("Robot Pose: " + follower.getPose().toString());

        telemetryM.update();
    }
}