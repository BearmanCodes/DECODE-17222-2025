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
public class ShootingHopperTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Follower follower;

    public static int L_VEL = 1000;

    public static double LAUNCHER_POS = 0.45;

    public static int R_VEL = 1000;

    public static double startPoseX = 55.92558139534884;

    public static double startPoseY = 8.037209302325575;

    public static double startPoseHeading = Math.toRadians(180);

    private final Pose startPose = new Pose(startPoseX, startPoseY, startPoseHeading); // Start Pose of our rob

    Telemetry dashTele = dashboard.getTelemetry();

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    @Override
    public void runOpMode() throws InterruptedException {
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        waitForStart();
        while (!isStopRequested()){
            dashTele.addData("Pose X: ", follower.getPose().getX());
            dashTele.addData("Pose Y: ", follower.getPose().getY());
            dashTele.addData("Pose Heading: ", follower.getPose().getHeading());
            dashTele.update();
            follower.update();
            shooterAutoCore.setLauncherPos(LAUNCHER_POS);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            if (gamepad1.aWasPressed()) {
                shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
                shooterAutoCore.setCRPower(1, dashTele);
                while (!isStopRequested() && !shooterAutoCore.shoot(3, dashTele)) {

                };
            }
        }
    }
}
