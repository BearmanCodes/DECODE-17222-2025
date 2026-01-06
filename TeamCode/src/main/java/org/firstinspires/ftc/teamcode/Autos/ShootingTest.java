package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous
public class ShootingTest extends LinearOpMode {

    public Follower follower;

    public static double startPoseX = 61.4605376476378;

    public static double startPoseY = 15.104509488804135;

    public static double startPoseHeading = 2.0019333362579346;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseHeading)); // Start Pose of our robot.

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int L_VEL = 1000;

    public static int R_VEL = 1000;

    public static double LAUNCHER_POS = 0.55;

    Telemetry dashTele = dashboard.getTelemetry();

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    @Override
    public void runOpMode() throws InterruptedException {
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        waitForStart();
        //shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow + ShooterAutoCore.KICK_ITERATOR);
        while (!isStopRequested()){
            dashTele.addData("Pose X: ", follower.getPose().getX());
            dashTele.addData("Pose Y: ", follower.getPose().getY());
            dashTele.addData("Pose Heading: ", follower.getPose().getHeading());
            dashTele.update();
            follower.update();
            shooterAutoCore.setLauncherPos(LAUNCHER_POS);
            shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            if (gamepad1.aWasPressed()) {
                shooterAutoCore.setCRPower(1, dashTele);
                shooterAutoCore.in();
                while (!isStopRequested() && !shooterAutoCore.intakeShoot(3, dashTele)) {
                    shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
                }
            }
        }
    }
}
