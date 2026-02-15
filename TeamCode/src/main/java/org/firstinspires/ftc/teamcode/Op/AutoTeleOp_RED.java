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

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));

    public static Pose startingPose;

    public static double DRIVE_SHOOT_REDUCER_COEFFICENT = 0.425;

    public static double AUTO_REDUCER = 0.65;

    public static DcMotorEx intake;

    public static double intakeReducer = 0.75;

    public static double driveReducer = 1;

    public static boolean isReduced = false;

    public static double RESET_HEADING_DEG = 180;

    public static double ALLOWED_HEADING_ERROR_DEG = 0.15;

    boolean wasMoved = false;

    public static double PEDRO_STANDING_HEADING_CONSTRAINT = Math.toDegrees(0.007);

    public static boolean automatedDrive = false;

    public static boolean useDefaultPose = false;

    public static Pose targetPose;

    public static Pose holdingPose;

    public Limelight3A limelight;


    public static double FAILSAFE_STICK_TRIGGER = 0.1;

    public static double X_TOLERANCE = .85;
    public static double Y_TOLERANCE = .85;
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

    public static boolean BLUE_ALLIANCE = false;

    public static boolean INTAKE_RUN = false;

    public static ModeCore.ALLIANCE currAlliance;

    public static double INTAKE_THRESHOLD = 0.15;

    public static boolean IS_ROBOT_CENTRIC = true;

    boolean firstEntry = true;
    boolean currentlyStill = false;
    boolean previouslyStill = false;
    boolean currentlyMoved = false;
    boolean previouslyMoved = false;

    private static PathChain targetPath;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        shooterCore.setCRPower(-1);
    }

    public void updatePose(){
        follower.update();
        PoseStorage.currentPose = follower.getPose();
    }

    @Override
    public void loop() {
        telemetry.addData("Current Mode: ", ModeCore.currentDriveMode);
        updatePose();
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
                setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
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
                setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE() || !follower.isBusy()) {
                    follower.setMaxPower(1);
                    isReduced = false;
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
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
            case HOLD:
                setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.RED);
                follower.holdPoint(getPoseToHold());
                boolean still = follower.atPose(getPoseToHold(), X_TOLERANCE, Y_TOLERANCE, Math.toRadians(HEADING_TOLERANCE_DEG));
                updateMovedState(still);
                if (currentlyMoved && !previouslyMoved){
                    shooterCore.stop_shoot_once();
                    gamepad2.rumbleBlips(1);
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
    }

    public void handleShootingInputs(){
        if (gamepad2.dpadDownWasPressed()) {
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

    private void setGamepadLeds(GAMEPAD_COLORS oneColor, GAMEPAD_COLORS twoColor){
        if (oneColor == GAMEPAD_COLORS.RED) {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == GAMEPAD_COLORS.BLUE) {
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == GAMEPAD_COLORS.GREEN) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.RED) {
            gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.BLUE) {
            gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updatePathToFollow(){
        targetPath = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, targetPose))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();
    }

    private void resetHeading() {
        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(RESET_HEADING_DEG)));
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
            INTAKE_RUN = !INTAKE_RUN;
            if (INTAKE_RUN) {
                inPower = intakeReducer;
            } else {
                inPower = 0;
            }
        }
        if (gamepad1.rightBumperWasPressed()) {
            INTAKE_RUN = !INTAKE_RUN;
            if (INTAKE_RUN) {
                inPower = -intakeReducer;
            } else {
                inPower = 0;
            }
        }
        intake.setPower(inPower);
    }
}
