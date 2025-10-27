package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriverControls {
    private final Follower follower;
    private final TelemetryManager telemetryM;
    private final DcMotor leftFront, leftBack, rightFront, rightBack;

    private static final double NORMAL_SPEED = 0.55;
    private static final double FULL_SPEED = 1.0;

    public DriverControls(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        this.telemetryM = telemetryM;

        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions for mecanum drive
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose());
        follower.update();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        follower.update();

        // Drivetrain controls
        double axial = -gamepad1.left_stick_y;   // Forward/backward
        double lateral = gamepad1.left_stick_x;  // Strafe left/right
        double yaw = gamepad1.right_stick_x;     // Turn/rotate

        // Apply speed scaling based on right bumper
        double speedMultiplier = gamepad1.right_bumper ? FULL_SPEED : NORMAL_SPEED;
        axial *= speedMultiplier;
        lateral *= speedMultiplier;
        yaw *= speedMultiplier;

        // Calculate motor powers using mecanum drive kinematics
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
    }

    public void updateTelemetry(Gamepad gamepad1) {
        if (gamepad1.right_bumper) {
            telemetryM.debug("Drive Mode: Full Speed (100%)");
        } else {
            telemetryM.debug("Drive Mode: Normal Speed (55%)");
        }
        telemetryM.debug("Robot Pose: " + follower.getPose().toString());
    }

    public Follower getFollower() {
        return follower;
    }
}