package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "CloseRedGate", group = "Auto")
public class CloseRedGate extends CloseGateBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}