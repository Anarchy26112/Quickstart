package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.BaseAuto.CloseSoloBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "CloseBlueSolo", group = "Auto")
public class CloseBlueSolo extends CloseSoloBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}