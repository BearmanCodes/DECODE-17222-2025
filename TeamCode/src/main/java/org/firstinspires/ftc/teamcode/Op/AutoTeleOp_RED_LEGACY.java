package org.firstinspires.ftc.teamcode.Op;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp
public class AutoTeleOp_RED_LEGACY extends OpMode {
    public static Follower follower;

    public static double inPower;

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));

    public static Pose startingPose;

    public static DcMotorEx intake;

    public static double intakeReducer = 0.15;

    public static boolean automatedDrive = false;

    public static boolean useDefaultPose = false;

    public static Pose targetPose;

    public static double FAILSAFE_STICK_TRIGGER = 0.5;

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

    public static boolean inFWD = false;

    public static boolean BLUE_ALLIANCE = false;

    public static ModeCore.ALLIANCE currAlliance;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        currAlliance = BLUE_ALLIANCE ? ModeCore.ALLIANCE.BLUE : ModeCore.ALLIANCE.RED;
        intake.setDirection(inDir);
        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;

        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        TempShooterAutoCore.init(hardwareMap);
        follower.update();
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetPose.getHeading() + Math.toRadians(PATH_HEADING_OFFSET), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        dashTele.addData("Current Mode: ", ModeCore.currentDriveMode);
        dashTele.update();
        telemetry.update();
        //ModeCore.autoShootHandler(gamepad2, currAlliance);
        if (gamepad2.rightBumperWasPressed()) {
            //ModeCore.deliveryCurrentMethod = ModeCore.BALL_DELIVERY_METHOD.HOPPER;
            TempShooterAutoCore.setLauncherPos(ModeCore.HOPPER_LOAD_PLATFORM_HEIGHT);
            TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
        }
        if (gamepad2.leftBumperWasPressed()) {
            //ModeCore.deliveryCurrentMethod = ModeCore.BALL_DELIVERY_METHOD.INTAKE;
            //TempShooterAutoCore.setLauncherPos(ModeCore.INTAKE_LOAD_PLATFORM_HEIGHT);
            //TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_INTAKE_LOAD);
        }
        if (gamepad2.dpadUpWasPressed()){
            ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.SHOOT_MODE;
            setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(1);
        }
        if (gamepad1.dpadUpWasPressed()) {
            luigiServoIntakeOffset += 0.01;
            double currPos = Math.round(TempShooterAutoCore.luigiServo.getPosition() * 100.00) / 100.00;
            TempShooterAutoCore.luigiServo.setPosition(currPos + 0.01);
            dashTele.update();
        }
        if (gamepad1.dpadDownWasPressed()) {
            luigiServoIntakeOffset -= 0.01;
            double currPos = Math.round(TempShooterAutoCore.luigiServo.getPosition() * 100.00) / 100.00;
            TempShooterAutoCore.luigiServo.setPosition(currPos - 0.01);
            dashTele.update();
        }
        if (gamepad1.rightBumperWasPressed()) {
            PATH_HEADING_OFFSET -= 1;
            follower.turn(Math.toRadians(1), false);
        }
        if (gamepad1.leftBumperWasPressed()) {
            PATH_HEADING_OFFSET += 1;
            follower.turn(Math.toRadians(1), true);
        }
        if (gamepad2.circleWasPressed()) {
            TempShooterAutoCore.setCRPower(0);
        }
        if (gamepad2.crossWasPressed()) {
            TempShooterAutoCore.setCRPower(1);
        }
        if (gamepad2.triangleWasPressed()) {
            TempShooterAutoCore.setCRPower(-1);
        }
        if (gamepad1.crossWasPressed()) {
            TempShooterAutoCore.spinUpFlys(0, 0);
        }
        
        switch (ModeCore.currentDriveMode){
            case MANUAL_DRIVE:
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                inPower = gamepad1.left_trigger - gamepad1.right_trigger;
                intake.setPower(inPower * intakeReducer);
                setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
                telemetry.addData("Automated Drive: ", automatedDrive);
                telemetry.update();
                if (gamepad2.startWasPressed()) {
                    if (targetPose != null){
                        gamepad1.rumbleBlips(3);
                        gamepad2.rumbleBlips(3);
                        follower.followPath(pathChain.get());
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.AUTOMATED_DRIVE;
                        break;
                    }
                    else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                break;
            case AUTOMATED_DRIVE:
                setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
                telemetry.addData("Automated Drive: ", automatedDrive);
                telemetry.update();
                if (gamepad1.circleWasPressed() || gamepad2.shareWasPressed() || STICK_PANIC_FAILSAFE()){
                    //TempShooterAutoCore.spinUpFlys(0, 0);
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                if (gamepad2.startWasPressed()) {
                    if (targetPose != null){
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);
                        follower.followPath(pathChain.get());
                    }
                    else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                if (gamepad2.dpadUpWasPressed()){
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.SHOOT_MODE;
                    break;
                }
                break;
            case SHOOT_MODE:
                /*
                if (ModeCore.deliveryCurrentMethod == ModeCore.BALL_DELIVERY_METHOD.HOPPER){
                    if (gamepad2.dpadDownWasPressed()) {
                        TempShooterAutoCore.shoot_RED(telemetry);
                        follower.update();
                        dashTele.update();
                    }
                    TempShooterAutoCore.RED_SURGE(telemetry);
                }
                */
                if (gamepad1.circleWasPressed() || gamepad2.shareWasPressed() || STICK_PANIC_FAILSAFE()) {
                    TempShooterAutoCore.setCRPower(0);
                    //TempShooterAutoCore.spinUpFlys(0, 0);
                    TempShooterAutoCore.shotsTaken = 0;
                    TempShooterAutoCore.canAddShot = true;
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                    ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                    break;
                }
                if (gamepad2.startWasPressed()) {
                    if (targetPose != null){
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);
                        follower.followPath(pathChain.get());
                    }
                    else {
                        gamepad2.rumbleBlips(4);
                    }
                }
                /*
                if (ModeCore.deliveryCurrentMethod == ModeCore.BALL_DELIVERY_METHOD.INTAKE){
                    if (gamepad2.dpadDownWasPressed()) {
                        TempShooterAutoCore.intake_SHOOT(telemetry, ModeCore.servoPosition + luigiServoIntakeOffset);
                        follower.update();
                        dashTele.update();
                    }
                    TempShooterAutoCore.intake_SURGE(telemetry, ModeCore.servoPosition);
                    if (gamepad1.circleWasPressed() || gamepad2.shareWasPressed() || STICK_PANIC_FAILSAFE()) {
                        //TempShooterAutoCore.spinUpFlys(0, 0);
                        TempShooterAutoCore.setCRPower(0);
                        TempShooterAutoCore.shotsTaken = 0;
                        TempShooterAutoCore.canAddShot = true;
                        follower.startTeleOpDrive(true);
                        gamepad1.rumbleBlips(3);
                        gamepad2.rumbleBlips(3);
                        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
                        break;
                    }
                    break;
                }
                 */
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
}
