package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Pusher {
    private final Servo pusherServo;
    private final Telemetry telemetry;
    private final ElapsedTime timer;

    // State machine
    private enum PusherState {
        RETRACTED,      // Pusher is retracted (home position), ready to push
        EXTENDING,      // Pusher is extending to push sample
        RETRACTING      // Pusher is retracting back to home position
    }

    private PusherState state = PusherState.RETRACTED;

    public Pusher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.timer = new ElapsedTime();
        this.pusherServo = hardwareMap.get(Servo.class, HW_PUSHER);
    }

    public void push() {
        if (state == PusherState.RETRACTED) {
            extend();
            state = PusherState.EXTENDING;
            timer.reset();
        }
    }

    // Update state machine
    public void update() {
        double elapsed = timer.milliseconds();

        switch (state) {
            case EXTENDING:
                if (elapsed >= PUSHER_PUSH_DURATION_MS) {
                    retract();
                    state = PusherState.RETRACTING;
                    timer.reset();
                }
                break;

            case RETRACTING:
                if (elapsed >= PUSHER_RETRACT_DELAY_MS) {
                    state = PusherState.RETRACTED;
                }
                break;

            case RETRACTED:
                // Waiting for push command, do nothing
                break;
        }
    }

    private void extend() {
        if (pusherServo != null) {
            pusherServo.setPosition(PUSHER_EXTENDED_POS);
        }
    }

    private void retract() {
        if (pusherServo != null) {
            pusherServo.setPosition(PUSHER_RETRACTED_POS);
        }
    }

    // Check if pusher is ready to get pushed
    public boolean isReady() {
        return state == PusherState.RETRACTED;
    }

    // Current state name
    public String getState() {
        return state.toString();
    }

    // Servo pos (0 - 1)
    public double getServoPosition() {
        return (pusherServo != null) ? pusherServo.getPosition() : -1.0;
    }

    // Emergency stop - immediately retract and reset
    public void stop() {
        retract();
        timer.reset();
        state = PusherState.RETRACTED;
    }
}