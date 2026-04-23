package org.firstinspires.ftc.teamcode.Robot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helpers.ButtonHelper;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

@TeleOp(name = "Shooter Tuning Test", group = "Test")
public class ShooterTuningTeleop extends LinearOpMode {

    private Shooter shooter;

    // Tunable values
    private double kV = 0.00037;
    private double kS = 0.02;
    private double kP = 0.0027;
    private double kD = 0.00012; // 0.00008

    private double targetVelocity = 1500;

    private boolean shooterOn = false;

    // Parameter selection
    // 0 = kV, 1 = kS, 2 = kP
    private int selectedParam = 0;

    private double coarseStep = 0.001;
    private double fineStep = 0.0001;

    // Button helpers
    private ButtonHelper a = new ButtonHelper();
    private ButtonHelper b = new ButtonHelper();
    private ButtonHelper x = new ButtonHelper();
    private ButtonHelper y = new ButtonHelper();

    private ButtonHelper dpadUp = new ButtonHelper();
    private ButtonHelper dpadDown = new ButtonHelper();
    private ButtonHelper dpadLeft = new ButtonHelper();
    private ButtonHelper dpadRight = new ButtonHelper();

    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap, telemetry);
        // shooter.setTunings(kV, kS, kP, kD);

        telemetry.addLine("Shooter Tuning Ready");
        telemetry.addLine("A = toggle shooter");
        telemetry.addLine("X/B = velocity + / -");
        telemetry.addLine("Dpad L/R = select kV / kS / kP");
        telemetry.addLine("Dpad U/D = adjust selected value");
        telemetry.addLine("Hold LB for fine tuning");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            handleButtons();

            // shooter.setTunings(kV, kS, kP, kD);

            if (shooterOn) {
                shooter.setVelocity(targetVelocity);
            } else {
                shooter.stop();
            }

            // shooter.update();

            sendTelemetry();
        }
    }

    private void handleButtons() {

        double step = gamepad1.left_bumper ? fineStep : coarseStep;

        if (a.wasPressed(gamepad1.a)) {
            shooterOn = !shooterOn;
        }

        if (x.wasPressed(gamepad1.x)) {
            targetVelocity += 50;
        }

        if (b.wasPressed(gamepad1.b)) {
            targetVelocity = Math.max(0, targetVelocity - 50);
        }

        if (y.wasPressed(gamepad1.y)) {
            shooterOn = false;
            targetVelocity = 1500;
        }

        // Parameter select
        if (dpadLeft.wasPressed(gamepad1.dpad_left)) {
            selectedParam--;
            if (selectedParam < 0) selectedParam = 2;
        }

        if (dpadRight.wasPressed(gamepad1.dpad_right)) {
            selectedParam++;
            if (selectedParam > 2) selectedParam = 0;
        }

        // Adjust value
        if (dpadUp.wasPressed(gamepad1.dpad_up)) {
            adjust(step);
        }

        if (dpadDown.wasPressed(gamepad1.dpad_down)) {
            adjust(-step);
        }
    }

    private void adjust(double delta) {

        switch (selectedParam) {

            case 0:
                kV = Math.max(0, kV + delta);
                break;

            case 1:
                kS = Math.max(0, kS + delta);
                break;

            case 2:
                kP = Math.max(0, kP + delta);
                break;
        }
    }

    private void sendTelemetry() {

        telemetry.addLine("=== Shooter Tuning ===");

        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Target Velocity", targetVelocity);

        telemetry.addLine("");

        telemetry.addData("Selected Param",
                selectedParam == 0 ? "kV" :
                        selectedParam == 1 ? "kS" : "kP");

        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("kP", "%.6f", kP);

        telemetry.addLine("");

        telemetry.addData("Right Velocity", "%.1f", shooter.getRightVelocity());
        telemetry.addData("Left Velocity", "%.1f", shooter.getLeftVelocity());

        telemetry.addLine("");

        telemetry.addData("Step Mode",
                gamepad1.left_bumper ? "FINE" : "COARSE");

        telemetry.update();
    }
}