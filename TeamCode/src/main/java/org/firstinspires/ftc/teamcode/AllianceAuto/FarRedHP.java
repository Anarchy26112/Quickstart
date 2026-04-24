package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.BaseAuto.FarHPBase;
import org.firstinspires.ftc.teamcode.BaseAuto.FarTripleBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "FarRedHP", group = "Auto")
public class FarRedHP extends FarHPBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}