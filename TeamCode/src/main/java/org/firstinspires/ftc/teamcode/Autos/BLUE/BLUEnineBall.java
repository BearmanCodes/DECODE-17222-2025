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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Op.DrivetrainCore;
import org.firstinspires.ftc.teamcode.Op.ModeCore;
import org.firstinspires.ftc.teamcode.Op.PoseStorage;
import org.firstinspires.ftc.teamcode.Op.PrismCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;


@Config
@Autonomous(name = "BLUE 9 BALL", group = "BLUE_FAR")
@Configurable // Panels
public class BLUEnineBall extends OpMode {
    public Limelight3A limelight;

    enum PATH_STATES {
        DRIVE_TO_FIRE_FROM_START,
        LL_ALIGN_FROM_START,
        FIRE_AFTER_START,
        GET_BALLS_1,
        DRIVE_TO_FIRE_FROM_BALLS_1,
        LL_ALIGN_FROM_BALLS_1,
        FIRE_AFTER_BALLS_1,
        GET_BALLS_2,
        DRIVE_TO_FIRE_FROM_BALLS_2,
        LL_ALIGN_FROM_BALLS_2,
        FIRE_AFTER_BALLS_2,
        DRIVE_TO_PARK_FROM_FIRE_2,
        END,
        FINISHED
    }

    public boolean isFirstSoShoot = true;

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore;
  public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
   private PATH_STATES pathState; // Current autonomous path state (state machine)

    PrismCore prismCore = new PrismCore();

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final Pose startPose = new Pose(BLUE_AUTO_CONSTANTS.STARTING_X, BLUE_AUTO_CONSTANTS.STARTING_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.STARTING_HEADING)); // Start Pose of our robot.
    private final Pose shootFar1 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_FAR_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.PICKUP_HEADING));
    private final Pose collectBalls1ControlPoint1 = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_CONTROL_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_CONTROL_Y);
    private final Pose shootFar2 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_FAR_2_HEADING));
    private final Pose shootFar3 = new Pose(BLUE_AUTO_CONSTANTS.SHOOT_FAR_3_POS_X, BLUE_AUTO_CONSTANTS.SHOOT_FAR_3_POS_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.SHOOT_FAR_3_HEADING));
    private final Pose parkingPose = new Pose(BLUE_AUTO_CONSTANTS.PARKING_X, BLUE_AUTO_CONSTANTS.PARKING_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.PARKING_HEADING));
    private final Pose collectBalls2 = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_2_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_2_Y, Math.toRadians(BLUE_AUTO_CONSTANTS.PICKUP_HEADING));
    private final Pose collectBalls2ControlPoint = new Pose(BLUE_AUTO_CONSTANTS.COLLECT_BALLS_2_CONTROL_X, BLUE_AUTO_CONSTANTS.COLLECT_BALLS_2_CONTROL_Y);

    DrivetrainCore dtCore = new DrivetrainCore();

    private PathChain startToFirePath, collect1Path, collect1ToFirePath, collect2Path, collect2ToFirePath, parkPath;

    private void setPathState(PATH_STATES pState) {
        pathState = pState;
        pathTimer.resetTimer();
        timer.reset();
    }

    @Override
  public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    dtCore.Init(hardwareMap);
    pathTimer.resetTimer();
    opmodeTimer.resetTimer();
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.start();
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    shooterAutoCore = new ShooterAutoCore(telemetry);

    shooterAutoCore.init(hardwareMap);
    shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
    shooterAutoCore.setCRPower(0, telemetry);

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);

    buildPaths();
    prismCore.Init(hardwareMap);

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
        timer.reset();
        shooterAutoCore.spinUpFlys(BLUE_AUTO_CONSTANTS.L_VEL, BLUE_AUTO_CONSTANTS.R_VEL);
        shooterAutoCore.setCRPower(-1, telemetry);
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
        shooterAutoCore.FlysPIDControl();
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
                setPathState(PATH_STATES.LL_ALIGN_FROM_START);
                break;
            case LL_ALIGN_FROM_START:
                if (!follower.isBusy()){
                    if (!limelightAlign()){
                        telemetry.update();
                    }
                    else{
                        setPathState(PATH_STATES.FIRE_AFTER_START);
                    }
                }
                break;
            case FIRE_AFTER_START:
                if (!follower.isBusy() && timer.time(TimeUnit.MILLISECONDS) > BLUE_AUTO_CONSTANTS.TIMEOUT){
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
                        prismCore.LL_BAD();
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
                    setPathState(PATH_STATES.LL_ALIGN_FROM_BALLS_1);
                }
                break;
            case LL_ALIGN_FROM_BALLS_1:
                if (!follower.isBusy()){
                    if (!limelightAlign()){
                        telemetry.update();
                    }
                    else{
                        setPathState(PATH_STATES.FIRE_AFTER_BALLS_1);
                    }
                }
                break;
            case FIRE_AFTER_BALLS_1:
                if (!follower.isBusy() && timer.time(TimeUnit.MILLISECONDS) > BLUE_AUTO_CONSTANTS.TIMEOUT) {
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
                        prismCore.LL_BAD();
                        setPathState(PATH_STATES.GET_BALLS_2);
                    }
                }
                break;
            case GET_BALLS_2:
                if (!follower.isBusy()) {
                    follower.followPath(collect2Path);
                    setPathState(PATH_STATES.DRIVE_TO_FIRE_FROM_BALLS_2);
                }
                break;
            case DRIVE_TO_FIRE_FROM_BALLS_2:
                if (!follower.isBusy()) {
                    follower.followPath(collect2ToFirePath);
                    setPathState(PATH_STATES.LL_ALIGN_FROM_BALLS_2);
                }
                break;
            case LL_ALIGN_FROM_BALLS_2:
                if (!follower.isBusy()){
                    if (!limelightAlign()){
                        telemetry.update();
                    }
                    else{
                        setPathState(PATH_STATES.FIRE_AFTER_BALLS_2);
                    }
                }
                break;
            case FIRE_AFTER_BALLS_2:
                if (!follower.isBusy() && timer.time(TimeUnit.MILLISECONDS) > BLUE_AUTO_CONSTANTS.TIMEOUT) {
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
                        setPathState(PATH_STATES.DRIVE_TO_PARK_FROM_FIRE_2);
                    }
                }
                break;
            case DRIVE_TO_PARK_FROM_FIRE_2:
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
                    shooterAutoCore.boot.setPower(0);
                    setPathState(PATH_STATES.FINISHED);
                }
                break;
        }
    }


    boolean limelightAlign(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            // - = ccw + = cw
            //cw is + - + -
            //ccw is - + - +
            double tx = llResult.getTx();
            double target = BLUE_AUTO_CONSTANTS.LIMELIGHT_TARGET;
            double deg_error = tx - target;
            boolean isLeft = deg_error < 0;
            if (Math.abs(deg_error) > BLUE_AUTO_CONSTANTS.      ALLOWED_HEADING_ERROR_DEG){
                if (isLeft) {
                    dtCore.setDrivetrainPower(-BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT);
                } else {
                    dtCore.setDrivetrainPower(BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -BLUE_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT);
                }
            } else {
                dtCore.setDrivetrainPower(0, 0, 0, 0);
                prismCore.LL_GOOD();
                gamepad2.rumbleBlips(2);
                return true;
            }
        }
        return false;
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

        collect2Path = follower.pathBuilder()
                .addPath(new BezierCurve(shootFar2, collectBalls2ControlPoint, collectBalls2))
                .setConstantHeadingInterpolation(collectBalls2.getHeading())
                .addParametricCallback(BLUE_AUTO_CONSTANTS.PICKUP_2_TEMPORAL, () -> follower.setMaxPower(BLUE_AUTO_CONSTANTS.PICKUP_POWER))
                .build();

        collect2ToFirePath = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls2, shootFar3))
                .setLinearHeadingInterpolation(collectBalls2.getHeading(), shootFar3.getHeading())
                .addParametricCallback(0.001, () -> follower.setMaxPower(1))
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFar3, parkingPose))
                .setLinearHeadingInterpolation(shootFar3.getHeading(), parkingPose.getHeading())
                .build();
  }
}
    