package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Op.ModeCore;
import org.firstinspires.ftc.teamcode.Op.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@Autonomous(name = "BLUE 6 CLOSE", group = "BLUE_CLOSE")
@Configurable // Panels
public class BLUEsixClose extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static long myEntryTime = 0;

    public static int PATH_INDEX = 1;
    Telemetry dashTele = dashboard.getTelemetry();

    public static int INTAKE_POWER_OFFSET = 100;

    public static double TIMEOUT = 1500;

    public static double INTAKE_TIMEOUT = 9000;

    public static int HEADING_OFFSET = 8;

    public static boolean HOLD_END = true;

    public static double PICKUP_POWER = 0.5; //0.4

    public static double ROLLBACK_POWER = 1;

    public boolean firstTimeCR = true;

    public boolean secondTimeCR = true;

    public boolean thirdTimeCR = true;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore(telemetry);
    public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)

    public static int L_VEL = 800;

    public static int R_VEL = 875;

    public static double SHOOT_CLOSE_POS_X = 63.5;
    public static double SHOOT_CLOSE_POS_Y = 83.5;

    public static double SHOOT_CLOSE_POS_HEADING = 139;

    public static double SHOOT_CLOSE_2_POS_X = 63.5;
    public static double SHOOT_CLOSE_2_POS_Y = 83.5;

    public static double SHOOT_CLOSE_2_HEADING = 139;

    public static double COLLECT_BALLS_X = 25;
    public static double COLLECT_BALLS_Y = 84.5;

    public static double COLLECT_BALLS_2_X = 19;

    public static double COLLECT_BALLS_2_Y = 84.5;

    public static double COLLECT_BALLS_2_CONTROL_X = 71.75;

    public static double COLLECT_BALLS_2_CONTROL_Y = 67.25;

    public static double PICKUP_1_TEMPORAL = 0.01;

    public static double PICKUP_2_TEMPORAL = 0.4333;

    private final Pose startPose = new Pose(15.5, 111, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose shootClose1 = new Pose(SHOOT_CLOSE_POS_X, SHOOT_CLOSE_POS_Y, Math.toRadians(SHOOT_CLOSE_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(COLLECT_BALLS_X, COLLECT_BALLS_Y, Math.toRadians(0));
    private final Pose shootClose2 = new Pose(SHOOT_CLOSE_2_POS_X, SHOOT_CLOSE_2_POS_Y, Math.toRadians(SHOOT_CLOSE_2_HEADING));

    private final Pose parkingPose = new Pose(130.5, 11.5, Math.toRadians(180));

    private final Pose collectBalls2 = new Pose(COLLECT_BALLS_2_X, COLLECT_BALLS_2_Y, Math.toRadians(0));

    private final Pose collectBalls2ControlPoint = new Pose(COLLECT_BALLS_2_CONTROL_X, COLLECT_BALLS_2_CONTROL_Y);

    private PathChain firstPath, collect1Path, goBack, shootThenCollect2, goBack2, shootThenPark;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop(){
        PoseStorage.currentPose = follower.getPose();
        shooterAutoCore.spinUpFlys(0, 0);
        dashTele.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        //shooterAutoCore.power_surge(150);
        telemetry.addData("Path State: ", pathState);
        // Log values to Panels and Driver Station
        dashTele.update();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                //shooterAutoCore.in();
                shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
                shooterAutoCore.setLauncherPos(ModeCore.BLUE_LINE_CLOSE_LAUNCHER);
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > TIMEOUT){
                    follower.followPath(collect1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > INTAKE_TIMEOUT) {
                    follower.followPath(goBack);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > TIMEOUT) {
                    follower.followPath(shootThenCollect2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shooterAutoCore.stop();
                    shooterAutoCore.setCRPower(0, telemetry);
                    PoseStorage.currentPose = follower.getPose();
                    dashTele.update();
                    setPathState(-1);
                }
                break;
        }
    }

    public void buildPaths(){
        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClose1.getHeading())
                .build();

        collect1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootClose1, collectBalls1))
                .setConstantHeadingInterpolation(collectBalls1.getHeading())
                .addCallback(FirstShoot)
                .addParametricCallback(PICKUP_1_TEMPORAL, () -> follower.setMaxPower(PICKUP_POWER))
                .build();

        goBack = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls1, shootClose2))
                .setLinearHeadingInterpolation(collectBalls1.getHeading(), shootClose2.getHeading())
                .addParametricCallback(0.001, () -> follower.setMaxPower(1))
                .build();

        shootThenCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootClose2, collectBalls2))
                .setConstantHeadingInterpolation(collectBalls2.getHeading())
                .addCallback(SecondShoot)
                .build();
    }
    PathCallback FirstShoot = new PathCallback() {
        @Override
        public boolean run() {
            follower.pausePathFollowing();
            while (!shooterAutoCore.close_shoot(3, dashTele)){
                dashTele.update();
            }
            shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
            shooterAutoCore.in();
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            follower.resumePathFollowing();
            return true;
        }

        @Override
        public void initialize() {
            if (firstTimeCR) {
                ShooterAutoCore.failsafeTimer.reset();
                shooterAutoCore.setCRPower(1, dashTele);
                shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                firstTimeCR = false;
            }
        }

        @Override
        public boolean isReady() {
            return true;
        }

        @Override
        public int getPathIndex() {
            return 0;
        }
    };

    PathCallback SecondShoot = new PathCallback() {
        @Override
        public boolean run() {
            follower.pausePathFollowing();
            while (!shooterAutoCore.close_shoot(3, dashTele)){
                dashTele.update();
            }
            shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
            shooterAutoCore.in();
            follower.setMaxPower(1);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            follower.resumePathFollowing();
            return true;
        }

        @Override
        public void initialize() {
            if (secondTimeCR) {
                ShooterAutoCore.failsafeTimer.reset();
                shooterAutoCore.setCRPower(1, dashTele);
                shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                secondTimeCR = false;
            }
        }

        @Override
        public boolean isReady() {
            return true;
        }

        @Override
        public int getPathIndex() {
            return 0;
        }
    };

    PathCallback ThirdShoot = new PathCallback() {
        @Override
        public boolean run() {
            follower.pausePathFollowing();
            while (!shooterAutoCore.close_shoot(3, dashTele)){
                dashTele.update();
            }
            shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
            shooterAutoCore.in();
            follower.setMaxPower(1);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            follower.resumePathFollowing();
            return true;
        }

        @Override
        public void initialize() {
            if (thirdTimeCR) {
                ShooterAutoCore.failsafeTimer.reset();
                shooterAutoCore.setCRPower(1, dashTele);
                shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                thirdTimeCR = false;
            }
        }

        @Override
        public boolean isReady() {
            return true;
        }

        @Override
        public int getPathIndex() {
            return 0;
        }
    };
}
    