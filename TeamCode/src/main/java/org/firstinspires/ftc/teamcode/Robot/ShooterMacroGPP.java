package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

public class ShooterMacroGPP extends ShooterMacroMotif {
    public ShooterMacroGPP(SpinDex spinDex, Shooter shooter, Pusher pusher, Telemetry telemetry) {
        super(spinDex, shooter, pusher, telemetry, new String[]{"g", "p", "p"});
    }
}
//TODO: there is really no need for these extra classes, better to keep code smaller and simpler