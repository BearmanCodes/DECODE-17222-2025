package org.firstinspires.ftc.teamcode.Temporary;


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
public class AutoTeleOp_BLUE extends OpMode {
    public static Follower follower;

    public static double inPower;

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));

    public static Pose startingPose;

    public static DcMotorEx intake;

    public static double intakeReducer = 0.15;

    public static boolean automatedDrive = false;

    public static Pose targetPose;

    public static double FAILSAFE_STICK_TRIGGER = 0.5;

    enum GAMEPAD_COLORS {
        RED,
        GREEN,
        BLUE
    }

    public static Supplier<PathChain> pathChain;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Telemetry dashTele = dashboard.getTelemetry();

    public static boolean inFWD = false;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        intake.setDirection(inDir);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(defaultStartingPose);
        TempShooterAutoCore.init(hardwareMap);
        follower.update();
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetPose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        follower.update();
        dashTele.update();
        telemetry.update();
        ModeCore.autoShootHandler(gamepad2, ModeCore.ALLIANCE.BLUE);
        if (gamepad2.rightBumperWasPressed()) {
            ModeCore.deliveryCurrentMethod = ModeCore.BALL_DELIVERY_METHOD.HOPPER;
            TempShooterAutoCore.setLauncherPos(ModeCore.HOPPER_LOAD_PLATFORM_HEIGHT);
            TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
        }
        if (gamepad2.leftBumperWasPressed()) {
            ModeCore.deliveryCurrentMethod = ModeCore.BALL_DELIVERY_METHOD.INTAKE;
            TempShooterAutoCore.setLauncherPos(ModeCore.INTAKE_LOAD_PLATFORM_HEIGHT);
            TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_INTAKE_LOAD);
        }

        if (!automatedDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            inPower = gamepad1.left_trigger - gamepad1.right_trigger;
            intake.setPower(inPower * intakeReducer);
            setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
            telemetry.addData("Automated Drive: ", automatedDrive);
            telemetry.update();
        }
        telemetry.addData("Automated Drive: ", automatedDrive);
        telemetry.update();

        if (gamepad2.startWasPressed()) {
            if (targetPose != null){
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }
            else {
                gamepad2.rumbleBlips(4);
            }
        }

        if (automatedDrive){
            setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
            telemetry.addData("Automated Drive: ", automatedDrive);
            telemetry.update();
            if (gamepad1.circleWasPressed()){
                follower.startTeleOpDrive(true);
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                automatedDrive = false;
            }
            if (gamepad2.dpadUpWasPressed()){
                if (ModeCore.deliveryCurrentMethod == ModeCore.BALL_DELIVERY_METHOD.HOPPER){
                    TempShooterAutoCore.setCRPower(1);
                    TempShooterAutoCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
                    while (!TempShooterAutoCore.shoot(3, dashTele)) {
                        setGamepadLeds(GAMEPAD_COLORS.RED, GAMEPAD_COLORS.GREEN);
                        follower.update();
                        dashTele.update();
                        if (STICK_PANIC_FAILSAFE()) {
                            TempShooterAutoCore.shotsTaken = 0;
                            follower.startTeleOpDrive(true);
                            gamepad1.rumbleBlips(3);
                            gamepad2.rumbleBlips(3);
                            automatedDrive = false;
                            break;
                        }
                    }
                    TempShooterAutoCore.shotsTaken = 0;
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                    automatedDrive = false;
                }
            }
        }
    }

    private boolean STICK_PANIC_FAILSAFE(){
        boolean one_left_y = Math.abs(gamepad1.left_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean one_left_x = Math.abs(gamepad1.left_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean one_right_y = Math.abs(gamepad1.right_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean one_right_x = Math.abs(gamepad1.right_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean two_left_y = Math.abs(gamepad2.left_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean two_left_x = Math.abs(gamepad2.left_stick_x) > FAILSAFE_STICK_TRIGGER;
        boolean two_right_y = Math.abs(gamepad2.right_stick_y) > FAILSAFE_STICK_TRIGGER;
        boolean two_right_x = Math.abs(gamepad2.right_stick_x) > FAILSAFE_STICK_TRIGGER;
        return one_left_y || one_left_x || one_right_y || one_right_x || two_left_y || two_left_x || two_right_y || two_right_x;
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
