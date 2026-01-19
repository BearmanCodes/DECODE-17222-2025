package org.firstinspires.ftc.teamcode.Autos.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import org.firstinspires.ftc.teamcode.Temporary.ModeCore;
import org.firstinspires.ftc.teamcode.Temporary.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@Autonomous(name = "RED Shooting PERF FAR", group = "RED")
@Configurable // Panels
public class REDshootingTEMPFAR extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static long myEntryTime = 0;

    public static int PATH_INDEX = 1;
    Telemetry dashTele = dashboard.getTelemetry();

    public static int INTAKE_POWER_OFFSET = 100;

    public static double TIMEOUT = 2200;

    public static int HEADING_OFFSET = 8;

    public static boolean HOLD_END = true;

    public static double PICKUP_POWER = 0.4;

    public static double ROLLBACK_POWER = 1;

    public boolean firstTimeCR = true;

    public boolean secondTimeCR = true;
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();
  public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
  private int pathState; // Current autonomous path state (state machine)

    public static int L_VEL = 975;

    public static int R_VEL = 1175;
    public static double SHOOT_FAR_POS_X = 84;
    public static double SHOOT_FAR_POS_Y = 12;

    public static double SHOOT_FAR_POS_HEADING = 69.5;

    public static double SHOOT_FAR_2_POS_X = 84;
    public static double SHOOT_FAR_2_POS_Y = 12.0;

    public static double SHOOT_FAR_2_HEADING = 65;

    public static double COLLECT_BALLS_X = 130.5;
    public static double COLLECT_BALLS_Y = 40;

    public static double COLLECT_BALLS_CONTROL_X = 73.059026559147;

    public static double COLLECT_BALLS_CONTROL_Y = 44;

    public static double PICKUP_POSE_X = 88;

    public static double PICKUP_POSE_Y = 30;

    public static double PICKUP_T_VALUE = 0.5;
    private final Pose startPose = new Pose(88.0744186, 8.037209302325575, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose shootFar1 = new Pose(SHOOT_FAR_POS_X, SHOOT_FAR_POS_Y, Math.toRadians(SHOOT_FAR_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(COLLECT_BALLS_X, COLLECT_BALLS_Y, Math.toRadians(180));
    private final Pose collectBalls1ControlPoint1 = new Pose(COLLECT_BALLS_CONTROL_X, COLLECT_BALLS_CONTROL_Y);
    private final Pose shootFar2 = new Pose(SHOOT_FAR_2_POS_X, SHOOT_FAR_2_POS_Y, Math.toRadians(SHOOT_FAR_2_HEADING));
    private final Pose parkingPose = new Pose(130.5, 11.5, Math.toRadians(180));

    private final Pose pickupPowerPose = new Pose(PICKUP_POSE_X, PICKUP_POSE_Y);
    private PathChain firstPath, collect1Path, goBack, shootThenPark;

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
                shooterAutoCore.setLauncherPos(ModeCore.RED_RIGHT_FAR_LAUNCHER);
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > TIMEOUT){
                    follower.followPath(collect1Path);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(ROLLBACK_POWER);
                    follower.followPath(goBack);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > TIMEOUT) {
                    follower.followPath(shootThenPark);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    PoseStorage.currentPose = follower.getPose();
                    dashTele.update();
                    setPathState(-1);
                }
                break;
        }
    }

  public void buildPaths(){
        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootFar1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootFar1.getHeading())
                .build();

        collect1Path = follower.pathBuilder()
                .addPath(new BezierCurve(shootFar1, collectBalls1ControlPoint1, collectBalls1))
                .setConstantHeadingInterpolation(collectBalls1.getHeading())
                .addCallback(FirstShoot)
                .addPoseCallback(pickupPowerPose, () -> follower.setMaxPower(PICKUP_POWER), PICKUP_T_VALUE)
                .build();

        goBack = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls1, shootFar2))
                .setLinearHeadingInterpolation(collectBalls1.getHeading(), shootFar2.getHeading())
                .build();

        shootThenPark = follower.pathBuilder()
                .addPath(new BezierLine(shootFar2, parkingPose))
                .setLinearHeadingInterpolation(shootFar2.getHeading(), parkingPose.getHeading())
                .addCallback(SecondShoot)
                .build();
  }
    PathCallback FirstShoot = new PathCallback() {
        @Override
        public boolean run() {
            follower.pausePathFollowing();
            while (!shooterAutoCore.shoot(3, dashTele)){
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
            while (!shooterAutoCore.shoot(3, dashTele)){
                dashTele.update();
            }
            shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
            shooterAutoCore.stop();
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
}
    