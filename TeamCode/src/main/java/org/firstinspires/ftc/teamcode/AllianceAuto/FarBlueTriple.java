package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.BaseAuto.FarTripleBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "FarBlueTriple", group = "Auto")
public class FarBlueTriple extends FarTripleBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}