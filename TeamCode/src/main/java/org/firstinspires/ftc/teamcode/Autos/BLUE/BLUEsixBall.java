package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Op.ModeCore;
import org.firstinspires.ftc.teamcode.Op.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@Autonomous(name = "BLUE 6 BALL", group = "BLUE_FAR")
@Configurable // Panels
public class BLUEsixBall extends OpMode {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum PATH_STATES {
        DRIVE_TO_FIRE_FROM_START,
        FIRE_AFTER_START,
        GET_BALLS_1,
        DRIVE_TO_FIRE_FROM_BALLS_1,
        FIRE_AFTER_BALLS_1,
        DRIVE_TO_PARK_FROM_FIRE_1,
        END,
        FINISHED
    }

    public boolean isFirstSoShoot = true;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();
    public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
    private PATH_STATES pathState; // Current autonomous path state (state machine)

    private final Pose startPose = new Pose(BLUE_AUTO_CONSTANTS.STARTING_X, BLUE_AUTO_CONSTANTS.STARTING_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.STARTING_HEADING)); // Start Pose of our robot.
    private final Pose shootFar1 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.PICKUP_HEADING));
    private final Pose collectBalls1ControlPoint1 = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_CONTROL_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_CONTROL_Y);
    private final Pose shootFar2 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_HEADING));
    private final Pose parkingPose = new Pose(BLUE_AUTO_CONSTANTS.PARKING_X, BLUE_AUTO_CONSTANTS.PARKING_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.PARKING_HEADING));
    private PathChain startToFirePath, collect1Path, collect1ToFirePath, parkPath;

    private void setPathState(PATH_STATES pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
        shooterAutoCore.setCRPower(0, telemetry);

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
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        shooterAutoCore.spinUpFlys(BLUE_AUTO_CONSTANTS.L_VEL, BLUE_AUTO_CONSTANTS.R_VEL);
        shooterAutoCore.setCRPower(-1, telemetry);
        setPathState(PATH_STATES.DRIVE_TO_FIRE_FROM_START);
    }

    public void updatePose(){
        follower.update();
        PoseStorage.currentPose = follower.getPose();
    }

    @Override
    public void loop() {
        updatePose();
        autonomousPathUpdate(); // Update autonomous state machine
        //shooterAutoCore.power_surge(150);
        telemetry.addData("Path State: ", pathState);
        // Log values to Panels and Driver Station
        telemetry.update();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState){
            case DRIVE_TO_FIRE_FROM_START:
                follower.followPath(startToFirePath);
                setPathState(PATH_STATES.FIRE_AFTER_START);
                break;
            case FIRE_AFTER_START:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > BLUE_AUTO_CONSTANTS.TIMEOUT){
                    if (isFirstSoShoot) {
                        ShooterAutoCore.failsafeTimer.reset();
                        shooterAutoCore.setCRPower(1, telemetry);
                        shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                        isFirstSoShoot = false;
                    }
                    if (!shooterAutoCore.shoot(3, telemetry)){
                        telemetry.update();
                    } else {
                        isFirstSoShoot = true;
                        setPathState(PATH_STATES.GET_BALLS_1);
                    }
                }
                break;
            case GET_BALLS_1:
                if (!follower.isBusy()) {
                    shooterAutoCore.in();
                    follower.followPath(collect1Path);
                    setPathState(PATH_STATES.DRIVE_TO_FIRE_FROM_BALLS_1);
                }
                break;
            case DRIVE_TO_FIRE_FROM_BALLS_1:
                if (!follower.isBusy()) {
                    follower.followPath(collect1ToFirePath);
                    setPathState(PATH_STATES.FIRE_AFTER_BALLS_1);
                }
                break;
            case FIRE_AFTER_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > BLUE_AUTO_CONSTANTS.TIMEOUT) {
                    if (isFirstSoShoot) {
                        ShooterAutoCore.failsafeTimer.reset();
                        shooterAutoCore.setCRPower(1, telemetry);
                        shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                        isFirstSoShoot = false;
                    }
                    if (!shooterAutoCore.shoot(3, telemetry)){
                        telemetry.update();
                    } else {
                        isFirstSoShoot = true;
                        setPathState(PATH_STATES.DRIVE_TO_PARK_FROM_FIRE_1);
                    }
                }
                break;
            case DRIVE_TO_PARK_FROM_FIRE_1:
                if (!follower.isBusy()) {
                    follower.followPath(parkPath);
                    setPathState(PATH_STATES.END);
                }
                break;
            case END:
                if (!follower.isBusy()){
                    shooterAutoCore.stop();
                    shooterAutoCore.setCRPower(0, telemetry);
                    PoseStorage.currentPose = follower.getPose();
                    shooterAutoCore.spinUpFlys(0, 0);
                    telemetry.update();
                    setPathState(PATH_STATES.FINISHED);
                }
                break;
        }
    }

    public void buildPaths(){
        startToFirePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootFar1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootFar1.getHeading())
                .build();

        collect1Path = follower.pathBuilder()
                .addPath(new BezierCurve(shootFar1, collectBalls1ControlPoint1, collectBalls1))
                .setConstantHeadingInterpolation(collectBalls1.getHeading())
                .addParametricCallback(BLUE_AUTO_CONSTANTS.PICKUP_1_TEMPORAL, () -> follower.setMaxPower(BLUE_AUTO_CONSTANTS.PICKUP_POWER))
                .build();

        collect1ToFirePath = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls1, shootFar2))
                .setLinearHeadingInterpolation(collectBalls1.getHeading(), shootFar2.getHeading())
                .addParametricCallback(0.001, () -> follower.setMaxPower(1))
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFar2, parkingPose))
                .setLinearHeadingInterpolation(shootFar2.getHeading(), parkingPose.getHeading())
                .build();
    }
}
    