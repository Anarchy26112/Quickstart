package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.BaseAuto.CloseSoloBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "CloseBlueGate", group = "Auto")
public class CloseBlueGate extends CloseGateBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}