package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.16047)
            .forwardZeroPowerAcceleration(-18.72715401749762)  // -33  lower reduces the tipping over
            .lateralZeroPowerAcceleration(-70.95477176292252)  // -60
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.03))
           // .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.01, .015))  //added  p-> 0.1
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0,0.05,0.03))
          //  .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5, 0, 0.08, 0.01))   //added p-> 2.5
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.4,0.0,0.03,0.6,0.5))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.001,0.6,0.01))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            2.0,
            1.0);

/*
    static {
        pathConstraints.setVelocityConstraint(0.1);  //0.025, default is 0.1
    }
*/

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.93481108898253) //depends on battery voltage, this is at 13V
            .yVelocity(62.17966587336982) //depends on battery voltage
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);
    
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)//-4.72
            .strafePodX(-1.0826671654)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

}
