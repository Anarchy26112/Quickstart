package org.firstinspires.ftc.teamcode.AllianceAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto.CloseGateBase;
import org.firstinspires.ftc.teamcode.BaseAuto.CloseSoloBase;
import org.firstinspires.ftc.teamcode.BaseAuto.OpenBase;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;

@Autonomous(name = "RedOpen", group = "Auto")
public class RedOpen extends OpenBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}