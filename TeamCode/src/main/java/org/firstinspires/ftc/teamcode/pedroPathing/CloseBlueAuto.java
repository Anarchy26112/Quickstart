package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;


import java.util.Locale;

@Autonomous(name = "CloseBlueAuto", group = "Auto")
public class CloseBlueAuto extends OpMode {

    // =========================
    // Pedro Pathing
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    // =========================
    // State tracking
    // =========================
    private int pathState;

    // =========================
    // Starting pose + path points
    // =========================
    private final Pose startPose = new Pose(119, 45, Math.toRadians(-40)); // close start, used to be 175,23, Math.toRadians(-135)

    public static Pose finalPose;

    // Shooting / look points
    private final Pose angle32Pt = new Pose(80, 19, Math.toRadians(47.5));
    private final Pose lookTag   = new Pose(80, 19, Math.toRadians(-20));

    // Optional "outshot" waypoints (were commented before; now built to avoid NPE)
    private final Pose OutShotZone  = new Pose(72, 20, Math.toRadians(90));
    private final Pose OutShotZone2 = new Pose(48, 20, Math.toRadians(90));
    private final Pose OutShotZone3 = new Pose(24, 20, Math.toRadians(90));

    // Collect points
    private final Pose startPt = new Pose(119, 45, Math.toRadians(-135));
    private final Pose firstTripleCollect      = new Pose(72, 31, Math.toRadians(90));
    private final Pose CollectedFirstTriple    = new Pose(72, 54, Math.toRadians(90));
    private final Pose secondTripleCollect     = new Pose(48, 31, Math.toRadians(90));
    private final Pose CollectedSecondTriple   = new Pose(48, 59, Math.toRadians(90));
    private final Pose CollectedSecondTriple2   = new Pose(48, 54, Math.toRadians(90));

    private final Pose thirdTripleCollect     = new Pose(24, 31, Math.toRadians(90));
    private final Pose CollectedThirdTriple   = new Pose(24, 59, Math.toRadians(90));
    private final Pose PushGatePt = new Pose(67,56, Math.toRadians(0));
    private final Pose EndPoint = new Pose(60,42, Math.toRadians(0));

    // Paths
    private PathChain parkoutsideshooting, parkoutsideshooting2, parkoutsideshooting3, parkoutsideshooting4;
    private PathChain angle32, LookAtAprilTag, PushGate, PushGateShoot;
    private PathChain goTocollectFirstTriple, IntakeFirstTriple, ShootFirstTriple;
    private PathChain goTocollectSecondTriple, IntakeSecondTriple, ShootSecondTriple, goBack2;
    private PathChain goTocollectThirdTriple, IntakeThirdTriple, ShootThirdTriple;

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("Follower", "Initialized");

        telemetry.addData("Subsystems", "Initialized");

        // Paths
        buildPaths();
        telemetry.addData("Paths", "Built");

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        // ✅ FIX: don't start in STOP state (-1). Start by moving to lookTag then scan.
        //✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅
        setPathState(-3);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Run auto state machine
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

        telemetry.addData("Follower Busy?", follower.isBusy());

        telemetry.update();

    }

    @Override
    public void stop() {
        if (follower != null) {
            PoseHandoff.save(follower.getPose());
        }

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // =========================
    // PATH BUILDING
    // =========================
    public void buildPaths() {

        // Drive to a look pose (so the camera can see tags reliably)
        LookAtAprilTag = follower.pathBuilder()
                .addPath(new BezierLine(startPt, lookTag))
                .setLinearHeadingInterpolation(startPt.getHeading(), lookTag.getHeading())
                .build();

        PushGate = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())
                .build();

        PushGateShoot = follower.pathBuilder()
                .addPath(new BezierLine(PushGatePt, angle32Pt))
                .setLinearHeadingInterpolation(PushGatePt.getHeading(), angle32Pt.getHeading())
                .build();
        goBack2 = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, CollectedSecondTriple2))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), CollectedSecondTriple2.getHeading())
                .build();


        // Move to shooting angle
        angle32 = follower.pathBuilder()
                .addPath(new BezierLine(startPt, angle32Pt))
                .setLinearHeadingInterpolation(startPt.getHeading(), angle32Pt.getHeading())
                .build();

        // Outshot zones (built so state machine never hits null)
        parkoutsideshooting = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone.getHeading())
                .build();

        parkoutsideshooting2 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone2))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone2.getHeading())
                .build();

        parkoutsideshooting4 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone3))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone3.getHeading())
                .build();

        parkoutsideshooting3 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, EndPoint))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), EndPoint.getHeading())
                .build();

        goTocollectFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone, firstTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone.getHeading(), firstTripleCollect.getHeading())
                .build();

        goTocollectSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone2, secondTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone2.getHeading(), secondTripleCollect.getHeading())
                .build();

        goTocollectThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone3, thirdTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone3.getHeading(), thirdTripleCollect.getHeading())
                .build();

        IntakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(firstTripleCollect, CollectedFirstTriple))
                .setLinearHeadingInterpolation(firstTripleCollect.getHeading(), CollectedFirstTriple.getHeading())
                .build();

        IntakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(secondTripleCollect, CollectedSecondTriple))
                .setLinearHeadingInterpolation(secondTripleCollect.getHeading(), CollectedSecondTriple.getHeading())
                .build();
        IntakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(thirdTripleCollect, CollectedThirdTriple))
                .setLinearHeadingInterpolation(thirdTripleCollect.getHeading(), CollectedThirdTriple.getHeading())
                .build();

        ShootFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())
                .build();

        ShootSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), angle32Pt.getHeading())
                .build();
        ShootThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedThirdTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedThirdTriple.getHeading(), angle32Pt.getHeading())
                .build();
    }

    // =========================
    // AUTO STATE MACHINE ✅
    // - scan motif in -2 (after driving to look pose in -3)
    // - shooter: use picker + active macro completion
    // - intake: timeout in cases 5 and 10
    // =========================
    public void autonomousPathUpdate() {
        switch (pathState) {
            // Go to shooting angle
            case 0:
                follower.followPath(angle32, true);
                setPathState(1);
                break;

            // Shoot preload using picker ✅
            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                       setPathState(2);
                    }

                }
                break;

            // Move out then to collect first triple
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting);
                    setPathState(-23);
                }
                break;
            case -23:

                setPathState(3);

                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectFirstTriple,1.0,true);
                    setPathState(4);
                }
                break;

            // Start intake + reset timeout clock ✅
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeFirstTriple, 0.4, true);
                    setPathState(5);
                }
                break;

            // Intake timeout handling ✅
            case 5:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            // Shoot after first triple using picker ✅
            case 7:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        setPathState(8);
                    }

                }
                break;

            // Move out to second outshot zone, then collect second triple
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting2);
                    setPathState(-24);
                }
                break;
            case -24:

                setPathState(9);

                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectSecondTriple,1.0,true);
                    setPathState(10);
                }
                break;

            // Start intake 2 + reset timeout clock ✅
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeSecondTriple, 0.4, true);

                    setPathState(11);
                }
                break;

            // Intake timeout 2 ✅
            case 11:
                if (!follower.isBusy()) {
                    setPathState(125);

                }
                break;

            case 125:
                if (!follower.isBusy()){
                    follower.followPath(ShootSecondTriple);
                    setPathState(12);
                }
                break;

            // Shoot after second triple using picker ✅
            case 12:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        setPathState(13);
                    }

                }
                break;

            // Park / end
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting4, 1.0, true);
                    setPathState(14);
                }
                break;

            case 14:


                setPathState(15);

                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectThirdTriple,1.0,true);
                    setPathState(16);
                }
                break;

            // Start intake 2 + reset timeout clock ✅
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeThirdTriple, 0.4, true);
                    setPathState(17);
                }
                break;

            // Intake timeout 2 ✅
            case 17:
                if (!follower.isBusy()) {
                    setPathState(-1);




                }
                break;

            // Shoot after second triple using picker ✅
            case 18:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        setPathState(-1);
                    }
                }
                break;

            case -1:
            default:
                // Done
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected Follower getFollower() {
        return follower;
    }
}