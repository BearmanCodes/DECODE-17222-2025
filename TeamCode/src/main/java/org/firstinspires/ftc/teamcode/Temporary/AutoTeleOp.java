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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Temporary.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import java.util.function.Supplier;

@Config
@TeleOp
public class AutoTeleOp extends OpMode {
    public static Follower follower;

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));

    public static Pose startingPose;

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


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == defaultStartingPose ? startingPose : PoseStorage.currentPose);
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
            setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
        }

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
                        //gamepad_led
                        follower.update();
                        dashTele.update();
                        if (STICK_PANIC_FAILSAFE()) break;
                    }
                    TempShooterAutoCore.shotsTaken = 0;
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(3);
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
        switch (oneColor) {
            case RED:
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            case BLUE:
                gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            case GREEN:
                gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        switch (twoColor) {
            case RED:
                gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            case BLUE:
                gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            case GREEN:
                gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
