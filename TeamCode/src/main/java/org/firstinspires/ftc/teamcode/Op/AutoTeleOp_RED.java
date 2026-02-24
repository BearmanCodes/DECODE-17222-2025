package org.firstinspires.ftc.teamcode.Op;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp
public class AutoTeleOp_RED extends OpMode {
    public static Follower follower;

    public static double inPower = 0;
    public static double STARTING_X = 88.0744186;

    public static double STARTING_Y = 8.037209302325575;
    public static double STARTING_HEADING = 0;

    public static double LIMELIGHT_TARGET = -1.5;
    public static Pose defaultStartingPose = new Pose(STARTING_X, STARTING_Y, Math.toRadians(STARTING_HEADING));

    public static Pose startingPose;

    public static double DRIVE_SHOOT_REDUCER_COEFFICENT = 0.2;

    public static double AUTO_REDUCER = 0.75; //0.65

    public static DcMotorEx intake;

    public static double intakeReducer = 0.75;

    public static double driveReducer = 1;

    public static boolean isReduced = false;

    public static double RESET_HEADING_DEG = 180;

    public static double ALLOWED_HEADING_ERROR_DEG = 0.285;

    boolean wasMoved = false;

    public static double PEDRO_STANDING_HEADING_CONSTRAINT = Math.toDegrees(0.007);

    public static boolean automatedDrive = false;

    public static boolean useDefaultPose = false;

    public static Pose targetPose;

    public static Pose holdingPose;

    public Limelight3A limelight;


    public static double FAILSAFE_STICK_TRIGGER = 0.1;

    public static double X_TOLERANCE = 2;
    public static double Y_TOLERANCE = 2;
    public static double HEADING_TOLERANCE_DEG = 5;

    enum GAMEPAD_COLORS {
        RED,
        GREEN,
        BLUE
    }

    public static double luigiServoIntakeOffset = 0;

    public static double PATH_HEADING_OFFSET = 0;

    public static Supplier<PathChain> pathChain;

    public static double lastSign;

    public static boolean inFWD = false;

    OpShooterCore shooterCore = new OpShooterCore(telemetry);
    public static double intakeGamepad;

    double previousLPM = 0;
    double previousRPM = 0;

    public static boolean BLUE_ALLIANCE = false;

    public static boolean INTAKE_RUN_FWD = false;

    public static boolean INTAKE_RUN_REV = false;

    boolean LIMELIGHT_ALIGNED = false;

    public static boolean BOOT_RUN = false;

    public static ModeCore.ALLIANCE currAlliance;

    public static double INTAKE_THRESHOLD = 0.15;

    public static boolean IS_ROBOT_CENTRIC = true;

    boolean firstEntry = true;
    boolean currentlyStill = false;
    boolean previouslyStill = false;
    boolean currentlyMoved = false;
    boolean previouslyMoved = false;

    private static PathChain targetPath;

    PrismCore prismCore = new PrismCore();

    DrivetrainCore dtCore = new DrivetrainCore();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prismCore.Init(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        currAlliance = BLUE_ALLIANCE ? ModeCore.ALLIANCE.BLUE : ModeCore.ALLIANCE.RED;
        intake.setDirection(inDir);
        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        shooterCore.init(hardwareMap);
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.constant(targetPose.getHeading() + Math.toRadians(PATH_HEADING_OFFSET)))
                .build();
        dtCore.Init(hardwareMap);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        prismCore.BAR_LIGHT();
        prismCore.LL_BAD();
        shooterCore.setCRPower(-1);
    }

    public void updatePose(){
        follower.update();
        PoseStorage.currentPose = follower.getPose();
    }

    @Override
    public void stop(){
        prismCore.CLEAR();
    }

    @Override
    public void loop() {
        telemetry.addData("Current Mode: ", ModeCore.currentDriveMode);
        shooterCore.FlysPIDControl();
        updatePose();
        shooterCore.updateColors();
        telemetry.addData("Vel: ", shooterCore.fly.getVelocity());
        telemetry.addData("Ver: ", shooterCore.fry.getVelocity());
        telemetry.addData("Desired Vel: ", shooterCore.load_fly_expected_vel());
        telemetry.addData("Desired Ver: ", shooterCore.load_fry_expected_vel());
        telemetry.addData("Follower X: ", follower.getPose().getX());
        telemetry.addData("Follower Y: ", follower.getPose().getY());
        telemetry.addData("Follower Heading: ", follower.getPose().getHeading());
        telemetry.update();
        ModeCore.autoShootHandler(gamepad2, currAlliance, shooterCore);
        shooterCore.shooting_loop();
        handleShootingInputs();
        storePositions();
        intakeControls();
        if (targetPose != null) {
            updatePathToFollow();
        }
        switch (ModeCore.currentDriveMode) {
            case MANUAL_DRIVE:
                shootingMoveReducer();
                follower.setTeleOpDrive(-gamepad1.left_stick_y * driveReducer, -gamepad1.left_stick_x * driveReducer, -gamepad1.right_stick_x * driveReducer, IS_ROBOT_CENTRIC);
                setGamepadLeds(AutoTeleOp_RED.GAMEPAD_COLORS.GREEN, AutoTeleOp_RED.GAMEPAD_COLORS.RED);
                if (gamepad2.startWasPressed()) {
                    if (targetPath != null) {
                        gamepad1.rumbleBlips(1);
                        follower.followPath(targetPath, AUTO_REDUCER, true);
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.AUTOMATED_DRIVE;
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                if (gamepad2.circleWasPressed()){
                    if (targetPose != null) {
                        gamepad1.rumbleBlips(1);
                        setPoseToHold(follower.getPose());
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.HOLD;
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                if (gamepad2.shareWasPressed()){
                    LLResult llResult = limelight.getLatestResult();
                    if (llResult != null && llResult.isValid()) {
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.LIMELIGHT;
                        prismCore.LL_BAD();
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                    break;
                }
                if (gamepad2.triangleWasPressed()){
                    if (targetPose != null) {
                        gamepad1.rumbleBlips(1);
                        setPoseToHold(targetPose);
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.HOLD;
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                break;
            case AUTOMATED_DRIVE:
                setGamepadLeds(AutoTeleOp_RED.GAMEPAD_COLORS.RED, AutoTeleOp_RED.GAMEPAD_COLORS.GREEN);
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE()) {
                    follower.setMaxPower(1);
                    isReduced = false;
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                if (!follower.isBusy()){
                    LLResult llResult = limelight.getLatestResult();
                    if (llResult != null && llResult.isValid()) {
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.LIMELIGHT;
                        prismCore.LL_BAD();
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                    break;
                }
                if (gamepad2.circleWasPressed()){
                    if (targetPose != null) {
                        gamepad1.rumbleBlips(1);
                        setPoseToHold(follower.getPose());
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.HOLD;
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                if (gamepad2.triangleWasPressed()){
                    if (targetPose != null) {
                        gamepad1.rumbleBlips(1);
                        setPoseToHold(targetPose);
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.HOLD;
                        break;
                    } else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                break;
            case LIMELIGHT:
                LLResult llResult = limelight.getLatestResult();
                if (llResult != null && llResult.isValid() && !LIMELIGHT_ALIGNED) {
                    // - = ccw + = cw
                    //cw is + - + -
                    //ccw is - + - +
                    double tx = llResult.getTx();
                    double target = LIMELIGHT_TARGET;
                    double deg_error = tx - target;
                    boolean isLeft = deg_error < 0;
                    if (Math.abs(deg_error) > ALLOWED_HEADING_ERROR_DEG){
                        if (isLeft) {
                            dtCore.setDrivetrainPower(-DRIVE_SHOOT_REDUCER_COEFFICENT, DRIVE_SHOOT_REDUCER_COEFFICENT, -DRIVE_SHOOT_REDUCER_COEFFICENT, DRIVE_SHOOT_REDUCER_COEFFICENT);
                        } else {
                            dtCore.setDrivetrainPower(DRIVE_SHOOT_REDUCER_COEFFICENT, -DRIVE_SHOOT_REDUCER_COEFFICENT, DRIVE_SHOOT_REDUCER_COEFFICENT, -DRIVE_SHOOT_REDUCER_COEFFICENT);
                        }
                    } else {
                        LIMELIGHT_ALIGNED = true;
                        gamepad2.rumbleBlips(2);
                        prismCore.LL_GOOD();
                    }
                }
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE()) {
                    follower.startTeleOpDrive(true);
                    follower.setMaxPower(1);
                    LIMELIGHT_ALIGNED = false;
                    prismCore.LL_BAD();
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                break;
            case HOLD:
                setGamepadLeds(AutoTeleOp_RED.GAMEPAD_COLORS.RED, AutoTeleOp_RED.GAMEPAD_COLORS.RED);
                follower.holdPoint(getPoseToHold());
                boolean still = follower.atPose(getPoseToHold(), X_TOLERANCE, Y_TOLERANCE, Math.toRadians(HEADING_TOLERANCE_DEG));
                updateMovedState(still);
                if (currentlyMoved && !previouslyMoved){
                    shooterCore.stop_shoot_once();
                    gamepad2.rumbleBlips( 1);
                }
                if (currentlyStill && !previouslyStill) {
                    gamepad2.rumbleBlips(2);
                }
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE()) {
                    if (!shooterCore.isShooting){
                        follower.setMaxPower(1);
                        isReduced = false;
                    }
                    follower.startTeleOpDrive(true);
                    gamepad2.rumbleBlips(3);
                    shooterCore.stop_shoot_once();
                    currentlyStill = false;
                    currentlyMoved = false;
                    previouslyStill = false;
                    previouslyMoved = false;
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                break;
        }
    }
    public void updateMovedState(boolean still){
        previouslyStill = currentlyStill;
        currentlyStill = still;
        previouslyMoved = currentlyMoved;
        currentlyMoved = !still;
        telemetry.addData("previouslyStill: ", previouslyStill);
        telemetry.addData("currentlyStill: ", currentlyStill);
        telemetry.addData("previouslyMoved: ", previouslyMoved);
        telemetry.addData("currentlyMoved: ", currentlyMoved);

    }

    public void handleShootingInputs(){
        boolean flysAreRunning = shooterCore.fly.getVelocity() >= OpShooterCore.RUNNING_SPEEDS && shooterCore.fry.getVelocity() >= OpShooterCore.RUNNING_SPEEDS;
        if (gamepad2.dpadDownWasPressed() && flysAreRunning) {
            shooterCore.start_shoot_once();
            isReduced = true;
        }
        if (gamepad2.dpadUpWasPressed()){
            shooterCore.stop_shoot_once();
            isReduced = false;
        }
    }

    private  boolean STICK_PANIC_FAILSAFE(){
        boolean one_left_y = Math.abs(gamepad1.left_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean one_left_x = Math.abs(gamepad1.left_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean one_right_y = Math.abs(gamepad1.right_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean one_right_x = Math.abs(gamepad1.right_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean two_left_y = Math.abs(gamepad2.left_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean two_left_x = Math.abs(gamepad2.left_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean two_right_y = Math.abs(gamepad2.right_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean two_right_x = Math.abs(gamepad2.right_stick_x) > FAILSAFE_STICK_TRIGGER;
        return one_left_y || one_left_x || one_right_y || one_right_x;
    }

    private void setGamepadLeds(AutoTeleOp_RED.GAMEPAD_COLORS oneColor, AutoTeleOp_RED.GAMEPAD_COLORS twoColor){
        if (oneColor == AutoTeleOp_RED.GAMEPAD_COLORS.RED) {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == AutoTeleOp_RED.GAMEPAD_COLORS.BLUE) {
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == AutoTeleOp_RED.GAMEPAD_COLORS.GREEN) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == AutoTeleOp_RED.GAMEPAD_COLORS.RED) {
            gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == AutoTeleOp_RED.GAMEPAD_COLORS.BLUE) {
            gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == AutoTeleOp_RED.GAMEPAD_COLORS.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updatePathToFollow(){
        targetPath = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }

    private void shootingMoveReducer(){
        if (gamepad1.leftStickButtonWasPressed()) {
            isReduced = !isReduced;
        }
        driveReducer = isReduced ? DRIVE_SHOOT_REDUCER_COEFFICENT : 1;
    }

    private void setPoseToHold(Pose poseToHold) {
        holdingPose = poseToHold;
    }

    private Pose getPoseToHold(){
        return holdingPose;
    }

    private void intakeControls(){
        if (gamepad1.leftBumperWasPressed()) {
            INTAKE_RUN_FWD = !INTAKE_RUN_FWD;
            INTAKE_RUN_REV = false;
            if (INTAKE_RUN_FWD) {
                inPower = intakeReducer;
                prismCore.INTAKE_SUCK();
                BOOT_RUN = true;
                shooterCore.boot_fwd();
            } else {
                inPower = 0;
                prismCore.INTAKE_NONE();
            }
        }
        if (gamepad1.rightBumperWasPressed()) {
            INTAKE_RUN_REV = !INTAKE_RUN_REV;
            INTAKE_RUN_FWD = false;
            if (INTAKE_RUN_REV) {
                inPower = -intakeReducer;
                prismCore.INTAKE_SPIT();
                BOOT_RUN = true;
                shooterCore.boot_rev();
            } else {
                inPower = 0;
                prismCore.INTAKE_NONE();
            }
        }
        if (gamepad1.crossWasPressed()) {
            BOOT_RUN = !BOOT_RUN;
            if (BOOT_RUN) {
                shooterCore.boot_fwd();
            } else {
                shooterCore.boot_stop();
            }
        }
        intake.setPower(inPower);
    }

    private void storePositions(){
        if (gamepad2.leftBumperWasPressed()){
            Pose currentPose = follower.getPose();
            PoseCore.RED_LINE_CLOSE_X = currentPose.getX();
            PoseCore.RED_LINE_CLOSE_Y = currentPose.getY();
            PoseCore.RED_LINE_CLOSE_HEADING = currentPose.getHeading();
            PoseCore.RED_LINE_CLOSE_POSE = new Pose(PoseCore.RED_LINE_CLOSE_X, PoseCore.RED_LINE_CLOSE_Y, PoseCore.RED_LINE_CLOSE_HEADING);
            targetPose = PoseCore.RED_LINE_CLOSE_POSE;
        }
        if (gamepad2.rightBumperWasPressed()){
            Pose currentPose = follower.getPose();
            PoseCore.RED_RIGHT_FAR_X = currentPose.getX();
            PoseCore.RED_RIGHT_FAR_Y = currentPose.getY();
            PoseCore.RED_RIGHT_FAR_HEADING = currentPose.getHeading();
            PoseCore.RED_RIGHT_FAR_POSE = new Pose(PoseCore.RED_RIGHT_FAR_X, PoseCore.RED_RIGHT_FAR_Y, PoseCore.RED_RIGHT_FAR_HEADING);
            targetPose = PoseCore.RED_RIGHT_FAR_POSE;
        }
    }
}
