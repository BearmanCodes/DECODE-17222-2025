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
@Autonomous(name = "BLUE 3 CLOSE", group = "BLUE_CLOSE")
@Configurable // Panels
public class BLUEthreeClose extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Telemetry dashTele = dashboard.getTelemetry();

    enum PATH_STATES {
        DRIVE_TO_FIRE_FROM_START,
        FIRE_AFTER_START,
        DRIVE_TO_PARK_FROM_FIRE,
        END,
        FINISHED,
    }

    public boolean isFirstSoShoot = true;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore(telemetry);
    public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
    private PATH_STATES pathState; // Current autonomous path state (state machine)

    public static int L_VEL = BLUE_AUTO_CONSTANTS.CLOSE_L_VEL;

    public static int R_VEL = BLUE_AUTO_CONSTANTS.CLOSE_R_VEL;

    private final Pose startPose = new Pose(BLUE_AUTO_CONSTANTS.CLOSE_STARTING_X, BLUE_AUTO_CONSTANTS.CLOSE_STARTING_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.CLOSE_STARTING_HEADING)); // Start Pose of our robot.
    private final Pose shootClose1 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_CLOSE_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_CLOSE_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_CLOSE_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose parkingPose = new Pose(BLUE_AUTO_CONSTANTS.CLOSE_PARK_X, BLUE_AUTO_CONSTANTS.CLOSE_PARK_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.CLOSE_PARK_HEADING));

    //private final Pose collectBalls2ControlPoint = new Pose(BLUE_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_CONTROL_X, BLUE_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_CONTROL_Y);

    private PathChain startToFirePath, parkFromFirePath;

    private void setPathState(PATH_STATES pState) {
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
        shooterAutoCore.setLauncherPos(ModeCore.BLUE_LINE_CLOSE_LAUNCHER);
        shooterAutoCore.boot.setPower(1);
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
        shooterAutoCore.FlysPIDControl();
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
            case DRIVE_TO_FIRE_FROM_START:
                follower.followPath(startToFirePath);
                setPathState(PATH_STATES.FIRE_AFTER_START);
                break;
            case FIRE_AFTER_START:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > BLUE_AUTO_CONSTANTS.CLOSE_TIMEOUT){
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
                        setPathState(PATH_STATES.DRIVE_TO_PARK_FROM_FIRE);
                    }
                }
                break;
            case DRIVE_TO_PARK_FROM_FIRE:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > BLUE_AUTO_CONSTANTS.INTAKE_TIMEOUT) {
                    follower.followPath(parkFromFirePath);
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
        }
    }

    public void buildPaths(){
        startToFirePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClose1.getHeading())
                .build();

        parkFromFirePath = follower.pathBuilder()
                .addPath(new BezierLine(shootClose1, parkingPose))
                .setConstantHeadingInterpolation(parkingPose.getHeading())
                .build();
    }
}
    