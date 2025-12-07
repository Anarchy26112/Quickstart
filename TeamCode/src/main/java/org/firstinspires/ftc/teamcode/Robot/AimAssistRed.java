package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.Kp_TURN;

import com.pedropathing.follower.Follower;

public class AimAssistRed {
    private Follower follower;
    private Limelight limelight;
    public AimAssistRed(Follower follower, Limelight limelight){
        this.follower = follower;
        this.limelight = limelight;
    }
    public double calulateTargetHeading(){
        double xDistance = (double) (138-follower.getPose().getX());
        double yDistance = (double) (138-follower.getPose().getY());
        return Math.atan(xDistance/yDistance);
    }
    public double getOdometryTurnPower(){
        double currentHeading = Math.toDegrees(follower.getHeading());
        double targetHeading = calulateTargetHeading();
        double error = targetHeading-currentHeading;
        return error*Kp_TURN;
    }
    public double getTurnPower(){
        limelight.setTargetRed();
        if(!limelight.isTargetVisible()){
            return getOdometryTurnPower();
        }
        else{
            return limelight.getTurnPower();
        }
    }

}
