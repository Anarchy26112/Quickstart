package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.Kp_TURN;

import com.pedropathing.follower.Follower;

public class AimAssistBlue {
    private Follower follower;
    private Limelight limelight;
    public AimAssistBlue(Follower follower, Limelight limelight){
        this.follower = follower;
        this.limelight = limelight;
    }
    public double calulateTargetHeading(){
        double xDistance = (double) (138-follower.getPose().getX());
        double yDistance = (double) (6-follower.getPose().getY());
        return Math.atan(yDistance/xDistance);
    }
    public double getOdometryTurnPower(){
        double currentHeading = Math.toDegrees(follower.getHeading());
        double targetHeading = calulateTargetHeading();
        double error = targetHeading-currentHeading;
        return error*Kp_TURN;
    }
    public double getTurnPower(){
        limelight.setTargetBlue();
        if(!limelight.isTargetVisible()){
            return getOdometryTurnPower();
        }
        else{
            return limelight.getTurnPower();
        }
    }

}
