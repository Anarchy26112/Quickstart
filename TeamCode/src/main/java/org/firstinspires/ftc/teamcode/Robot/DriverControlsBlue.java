package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NORMAL_SPEED;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

public class DriverControlsBlue {
    private Follower follower;
    private Telemetry telemetry;
    private Limelight limelight;

    private boolean slowMode = false;
    private boolean fastMode = true;
    private boolean autoAlignEnabled = false;
    private AimAssistBlue aimAssistBlue = new AimAssistBlue(follower, limelight);

    private final ButtonHelper btnTouchpad = new ButtonHelper();

    public DriverControlsBlue(HardwareMap hardwareMap, Telemetry telemetry, Limelight limelight) {
        this.telemetry = telemetry;
        this.limelight = limelight;

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        // Update follower (required every loop)
        follower.update();

        // Toggle auto-align with touchpad
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        // Toggle fast mode with left stick button
        fastMode = !gamepad1.left_stick_button;

        // Get base drive inputs
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // If auto-align is enabled and target is visible, override turn with Limelight
        if (autoAlignEnabled && limelight.isTargetVisible()) {
            turn = aimAssistBlue.getTurnPower();
        }

        if (fastMode) {
            // Full speed mode (100%)
            follower.setTeleOpDrive(drive, strafe, turn, true);
        } else {
            // Slow speed mode (55%)
            follower.setTeleOpDrive(
                    drive * NORMAL_SPEED,
                    strafe * NORMAL_SPEED,
                    turn * 0.45,
                    true
            );
        }
    }

    public void updateTelemetry() {
        if (fastMode) {
            telemetry.addData("Drive Mode", "Full Speed (100%)");
        } else {
            telemetry.addData("Drive Mode", "Slow Speed (55%)");
        }

        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");

        if (autoAlignEnabled && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag " + limelight.getDetectedTagId());
            telemetry.addData("Turn Power", "%.2f", limelight.getTurnPower());
        }

        telemetry.addData("X", String.format(Locale.US, "%.1f", follower.getPose().getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.1f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format(Locale.US, "%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Velocity", String.format(Locale.US, "%.2f", follower.getVelocity().getMagnitude()));
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }
}