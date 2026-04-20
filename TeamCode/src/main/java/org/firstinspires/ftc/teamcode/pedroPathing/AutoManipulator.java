package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

public class AutoManipulator {

    public enum State {
        IDLE,
        INTAKING,
        HOLDING,
        SHOOTING
    }

    private final Intake intake;
    private final Gate gate;
    private final Telemetry telemetry;

    private State currentState = State.IDLE;

    // =========================
    // Tunables
    // =========================
    private double intakePower = 1.0;
    private double intakeTransferPower = 0.67;

    private double holdingIntakePower = 1.0;
    private double holdingTransferPower = 0.0;

    private double shootingIntakePower = 1.0;
    private double shootingTransferPower = 1.0;

    private long shootingStartDelayMs = 0;
    private long shootingDurationMs = 550;

    // =========================
    // Internal timing
    // =========================
    private long stateStartTimeMs = 0;
    private long shootingRequestedAtMs = 0;
    private boolean waitingToFeed = false;

    public AutoManipulator(Intake intake,
                           Gate gate,
                           Telemetry telemetry) {
        this.intake = intake;
        this.gate = gate;
        this.telemetry = telemetry;

        setState(State.IDLE);
    }

    public void update() {
        if (currentState == State.SHOOTING
                && waitingToFeed
                && System.currentTimeMillis() - shootingRequestedAtMs >= shootingStartDelayMs) {

            intake.intake(shootingIntakePower);
            intake.transferIn(shootingTransferPower);
            waitingToFeed = false;
        }
    }

    public void setState(State newState) {
        if (newState == currentState) return;

        currentState = newState;
        stateStartTimeMs = System.currentTimeMillis();

        switch (newState) {
            case IDLE:
                waitingToFeed = false;
                shootingRequestedAtMs = 0;
                intake.stopAll();
                gate.block();
                break;

            case INTAKING:
                waitingToFeed = false;
                shootingRequestedAtMs = 0;
                gate.block();
                intake.intake(intakePower);
                intake.transferIn(intakeTransferPower);
                break;

            case HOLDING:
                waitingToFeed = false;
                shootingRequestedAtMs = 0;
                gate.block();
                intake.intake(holdingIntakePower);
                intake.transferIn(holdingTransferPower);
                break;

            case SHOOTING:
                gate.open();
                intake.stopAll();   // wait before feeding
                waitingToFeed = true;
                shootingRequestedAtMs = System.currentTimeMillis();
                break;
        }
    }

    public State getState() {
        return currentState;
    }

    public void intake() {
        setState(State.INTAKING);
    }

    public void hold() {
        setState(State.HOLDING);
    }

    // NEW: use this halfway to shooting pose
    public void releaseForShot() {
        waitingToFeed = false;
        intake.stopAll();
        gate.open();
    }

    public void shoot() {
        setState(State.SHOOTING);
    }

    public void idle() {
        setState(State.IDLE);
    }

    public boolean isShootComplete() {
        if (currentState != State.SHOOTING) return true;
        return System.currentTimeMillis() - stateStartTimeMs >= shootingDurationMs;
    }

    public boolean isWaitingToFeed() {
        return waitingToFeed;
    }

    public long getStateElapsedMs() {
        return System.currentTimeMillis() - stateStartTimeMs;
    }

    public void setIntakePower(double intakePower, double transferPower) {
        this.intakePower = intakePower;
        this.intakeTransferPower = transferPower;
    }

    public void setHoldingPower(double holdingIntakePower, double holdingTransferPower) {
        this.holdingIntakePower = holdingIntakePower;
        this.holdingTransferPower = holdingTransferPower;
    }

    public void setShootingFeedPower(double intakePower, double transferPower) {
        this.shootingIntakePower = intakePower;
        this.shootingTransferPower = transferPower;
    }

    public void setShootingTimings(long startDelayMs, long durationMs) {
        this.shootingStartDelayMs = startDelayMs;
        this.shootingDurationMs = durationMs;
    }

    public void addTelemetry() {
        telemetry.addData("Auto Manip State", currentState);
        telemetry.addData("Auto Manip State Time", getStateElapsedMs());
        telemetry.addData("Auto Waiting To Feed", waitingToFeed);
    }

    public void stopAll() {
        setState(State.IDLE);
    }
}