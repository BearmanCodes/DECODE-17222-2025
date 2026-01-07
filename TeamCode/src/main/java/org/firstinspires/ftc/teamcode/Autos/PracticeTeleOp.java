package org.firstinspires.ftc.teamcode.Autos;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp
public class PracticeTeleOp extends OpMode {

    private Follower follower;

    public static int L_VEL = 900;

    public static int R_VEL = 900;

    private final Pose startingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); // Start Pose of our robot.

    private final Pose shooting1Pose = new Pose(56.515188335358445, 88, Math.toRadians(135)); // Highest (First Set) of Artifacts from the Spike Mark.

    private boolean automatedDrive;

    private Supplier<PathChain> pathChain;

    private ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Telemetry dashTele = dashboard.getTelemetry();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        shooterAutoCore.init(hardwareMap);

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, shooting1Pose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, shooting1Pose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        dashTele.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        handleGamepadLED();

        if (gamepad2.bWasPressed()) {
            gamepad1.rumbleBlips(3);
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.circleWasPressed())) {
            follower.startTeleOpDrive(true);
            gamepad1.rumbleBlips(3);
            automatedDrive = false;
        }

        if (gamepad2.aWasPressed()) {
            shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            shooterAutoCore.setLauncherPos(ShooterAutoCore.laInitPos);
        }

        if (automatedDrive && gamepad2.yWasPressed()){
            shooterAutoCore.setCRPower(1, dashTele);
            shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
            while (!shooterAutoCore.shoot(3, dashTele)){
                handleGamepadLED();
                dashTele.update();
                if (gamepad1.circleWasPressed() || gamepad2.xWasPressed()) {
                    follower.startTeleOpDrive(true);
                    gamepad1.rumbleBlips(3);
                    automatedDrive = false;
                    break;
                }
            }
        }
    }

    public void handleGamepadLED(){
        if (automatedDrive) {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
