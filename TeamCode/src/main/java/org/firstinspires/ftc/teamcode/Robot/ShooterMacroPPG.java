package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

public class ShooterMacroPPG extends ShooterMacroMotif {
    public ShooterMacroPPG(SpinDex spinDex, Shooter shooter, Pusher pusher, Telemetry telemetry) {
        super(spinDex, shooter, pusher, telemetry, new String[]{"p", "g", "p"});
    }
}
