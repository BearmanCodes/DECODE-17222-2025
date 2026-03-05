package org.firstinspires.ftc.teamcode.Autos.RED;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Op.DrivetrainCore;
import org.firstinspires.ftc.teamcode.Op.ModeCore;
import org.firstinspires.ftc.teamcode.Op.PoseStorage;
import org.firstinspires.ftc.teamcode.Op.PrismCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;


@Config
@Autonomous(name = "RED 3 CLOSE", group = "RED_CLOSE")
@Configurable // Panels
public class REDthreeClose extends OpMode {

    public Limelight3A limelight;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Telemetry dashTele = dashboard.getTelemetry();

    enum PATH_STATES {
        DRIVE_TO_FIRE_FROM_START,
        LL_ALIGN_FROM_START,
        FIRE_AFTER_START,
        DRIVE_TO_PARK_AFTER_FIRE_START,
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

    PrismCore prismCore = new PrismCore();

    DrivetrainCore dtCore = new DrivetrainCore();

    public static int L_VEL = RED_AUTO_CONSTANTS.CLOSE_L_VEL;

    public static int R_VEL = RED_AUTO_CONSTANTS.CLOSE_R_VEL;

    private final Pose startPose = new Pose(RED_AUTO_CONSTANTS.CLOSE_STARTING_X, RED_AUTO_CONSTANTS.CLOSE_STARTING_Y, Math.toRadians(RED_AUTO_CONSTANTS.CLOSE_STARTING_HEADING)); // Start Pose of our robot.
    private final Pose shootClose1 = new Pose(RED_AUTO_CONSTANTS.SHOOT_CLOSE_POS_X, RED_AUTO_CONSTANTS.SHOOT_CLOSE_POS_Y, Math.toRadians(RED_AUTO_CONSTANTS.SHOOT_CLOSE_POS_HEADING)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_X, RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_Y, Math.toRadians(RED_AUTO_CONSTANTS.PICKUP_HEADING));
    private final Pose shootClose2 = new Pose(RED_AUTO_CONSTANTS.SHOOT_CLOSE_2_POS_X, RED_AUTO_CONSTANTS.SHOOT_CLOSE_2_POS_Y, Math.toRadians(RED_AUTO_CONSTANTS.SHOOT_CLOSE_2_HEADING));
    private final Pose collectBalls2 = new Pose(RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_X, RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_Y, Math.toRadians(RED_AUTO_CONSTANTS.PICKUP_HEADING));

    //private final Pose collectBalls2ControlPoint = new Pose(RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_CONTROL_X, RED_AUTO_CONSTANTS.CLOSE_COLLECT_BALLS_2_CONTROL_Y);

    private PathChain startToFirePath, parkFromFirePath;

    private void setPathState(PATH_STATES pState) {
        pathState = pState;
        pathTimer.resetTimer();
        timer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        dtCore.Init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);

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
        dashTele.update();
        prismCore.CLEAR();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        timer.reset();
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
                if (!follower.isBusy() && timer.time(TimeUnit.MILLISECONDS) > RED_AUTO_CONSTANTS.CLOSE_TIMEOUT){
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
                        setPathState(PATH_STATES.DRIVE_TO_PARK_AFTER_FIRE_START);
                    }
                }
                break;
            case DRIVE_TO_PARK_AFTER_FIRE_START:
                if (!follower.isBusy()){
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
                    prismCore.CLEAR();
                    setPathState(PATH_STATES.FINISHED);
                }
        }
    }

    boolean limelightAlign(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            // - = ccw + = cw
            //cw is + - + -
            //ccw is - + - +
            double tx = llResult.getTx();
            double target = RED_AUTO_CONSTANTS.CLOSE_LIMELIGHT_TARGET;
            double deg_error = tx - target;
            boolean isLeft = deg_error < 0;
            if (Math.abs(deg_error) > RED_AUTO_CONSTANTS.      ALLOWED_HEADING_ERROR_DEG){
                if (isLeft) {
                    dtCore.setDrivetrainPower(-RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT);
                } else {
                    dtCore.setDrivetrainPower(RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT, -RED_AUTO_CONSTANTS.DRIVE_SHOOT_REDUCER_COEFFICENT);
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
                .addPath(new BezierLine(startPose, shootClose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClose1.getHeading())
                .build();

        parkFromFirePath = follower.pathBuilder()
                .addPath(new BezierLine(shootClose2, collectBalls2))
                .setConstantHeadingInterpolation(collectBalls2.getHeading())
                .build();
    }
}
    