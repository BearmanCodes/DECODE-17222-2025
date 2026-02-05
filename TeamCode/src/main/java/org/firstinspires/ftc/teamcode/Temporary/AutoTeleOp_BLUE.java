package org.firstinspires.ftc.teamcode.Temporary;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Config
@TeleOp
public class AutoTeleOp_BLUE extends OpMode {
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

    public static double PEDRO_STANDING_HEADING_CONSTRAINT = Math.toDegrees(0.007);

    public static boolean automatedDrive = false;

    public static boolean useDefaultPose = false;

    public static Pose targetPose;

    public Limelight3A limelight;


    public static double FAILSAFE_STICK_TRIGGER = 0.1;

    public static double X_TOLERANCE = .35;
    public static double Y_TOLERANCE = .35;
    public static double HEADING_TOLERANCE_DEG = 3;

    enum GAMEPAD_COLORS {
        RED,
        GREEN,
        BLUE
    }

    public static double luigiServoIntakeOffset = 0;

    public static double PATH_HEADING_OFFSET = 0;

    public static Supplier<PathChain> pathChain;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Telemetry dashTele = dashboard.getTelemetry();

    public static double lastSign;

    public static boolean inFWD = false;

    public static double intakeGamepad;

    public static boolean BLUE_ALLIANCE = true;

    public static boolean INTAKE_RUN = false;


    public static ModeCore.ALLIANCE currAlliance;

    public static double INTAKE_THRESHOLD = 0.15;

    public static boolean IS_ROBOT_CENTRIC = true;

    private static PathChain targetPath;

    @Override
    public void init() {
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
        follower.pathConstraints.setHeadingConstraint(Math.toRadians(ALLOWED_HEADING_ERROR_DEG));
        TempShooterAutoCore.init(hardwareMap);
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.constant(targetPose.getHeading() + Math.toRadians(PATH_HEADING_OFFSET)))
                .build();
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Automated Drive: ", automatedDrive);
        follower.update();
        dashTele.update();
        telemetry.update();
        ModeCore.autoShootHandler(gamepad2, currAlliance);
        TempShooterAutoCore.RED_SURGE(telemetry);
        if (gamepad2.dpadDownWasPressed()) {
            TempShooterAutoCore.shoot_RED(telemetry);
            isReduced = true;
        }
        if (gamepad2.dpadUpWasPressed()){
            TempShooterAutoCore.stop_shooting();
            isReduced = false;
        }
        changeHeading();
        intakeControls();
        shootingMoveReducer();
        if (targetPose != null) {
            updatePathToFollow();
        }
        if (gamepad2.triangleWasPressed()){
            if (targetPose != null) {
                gamepad1.rumbleBlips(1);
                follower.holdPoint(targetPose);
                ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.HOLD;
            } else {
                gamepad1.rumbleBlips(4);
            }
        }
        switch (ModeCore.currentDriveMode) {
            case MANUAL_DRIVE:
                follower.setTeleOpDrive(-gamepad1.left_stick_y * driveReducer, -gamepad1.left_stick_x * driveReducer, -gamepad1.right_stick_x * driveReducer, IS_ROBOT_CENTRIC);
                setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
                if (gamepad2.startWasPressed()) {
                    if (targetPath != null) {
                        gamepad1.rumbleBlips(1);
                        follower.followPath(targetPath, AUTO_REDUCER, true);
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.AUTOMATED_DRIVE;
                        break;
                    } else {
                        gamepad1.rumbleBlips(4);
                    }
                }
                break;
            case AUTOMATED_DRIVE:
                setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE() || !follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                break;
            case HOLD:
                setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.RED);
                follower.holdPoint(targetPose);
                boolean notMoved = follower.atPose(targetPose, X_TOLERANCE, Y_TOLERANCE, Math.toRadians(HEADING_TOLERANCE_DEG));
                if (notMoved) {
                    TempShooterAutoCore.shoot_RED(telemetry);
                } else {
                    TempShooterAutoCore.stop_shooting();
                }
                if (gamepad1.circleWasPressed() || STICK_PANIC_FAILSAFE()) {
                    follower.setMaxPower(1);
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                    TempShooterAutoCore.stop_shooting();
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                break;
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

    private void setToLoadingBallsPosition(){
        TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
        TempShooterAutoCore.setCRPower(-1);
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

    private void changeHeading(){
        if (gamepad2.leftBumperWasPressed()){
            Pose currentPose = follower.getPose();
            PoseCore.BLUE_LINE_CLOSE_X = currentPose.getX();
            PoseCore.BLUE_LINE_CLOSE_Y = currentPose.getY();
            PoseCore.BLUE_LINE_CLOSE_HEADING = currentPose.getHeading();
            PoseCore.BLUE_LINE_CLOSE_POSE = new Pose(PoseCore.BLUE_LINE_CLOSE_X, PoseCore.BLUE_LINE_CLOSE_Y, PoseCore.BLUE_LINE_CLOSE_HEADING);
        }
        if (gamepad2.rightBumperWasPressed()){
            Pose currentPose = follower.getPose();
            PoseCore.BLUE_LEFT_FAR_X = currentPose.getX();
            PoseCore.BLUE_LEFT_FAR_Y = currentPose.getY();
            PoseCore.BLUE_LEFT_FAR_HEADING = currentPose.getHeading();
            PoseCore.BLUE_LEFT_FAR_POSE = new Pose(PoseCore.BLUE_LEFT_FAR_X, PoseCore.BLUE_LEFT_FAR_Y, PoseCore.BLUE_LEFT_FAR_HEADING);
        }
    }

    private void shootingMoveReducer(){
        if (gamepad1.leftStickButtonWasPressed()) {
            isReduced = !isReduced;
        }
        driveReducer = isReduced ? DRIVE_SHOOT_REDUCER_COEFFICENT : 1;
    }

    private void intakeControls(){
        if (gamepad1.leftBumperWasPressed()) {
            INTAKE_RUN = !INTAKE_RUN;
            if (INTAKE_RUN) {
                setToLoadingBallsPosition();
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
