package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class ShootingOPTest extends LinearOpMode {

    public static Follower follower;

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));

    public static Pose startingPose;

    public static boolean useDefaultPose = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int L_VEL = 2145;

    public static int R_VEL = 2575;

    OpShooterCore shooterAutoCore;

    public static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static boolean hasShot = false;

    public static double SURGE_TOLERANCE = 150;

    public static double MEET_SPEED_TOL = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAutoCore = new OpShooterCore(telemetry);
        shooterAutoCore.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        waitForStart();

        while (!isStopRequested()){
            follower.update();
            //shooterAutoCore.FlysPIDControl();
            shooterAutoCore.setFlySpeeds(L_VEL, R_VEL);
            shooterAutoCore.shooting_loop();
            shooterAutoCore.setLauncherPos(OpShooterCore.laInitPos);
            if (gamepad2.dpadDownWasPressed()) {
                shooterAutoCore.start_shoot_once();
            }
            if (gamepad2.dpadUpWasPressed()){
                shooterAutoCore.stop_shoot_once();
            }
            telemetry.addData("Follower X: ", follower.getPose().getX());
            telemetry.addData("Follower Y: ", follower.getPose().getY());
            telemetry.addData("Follower Heading: ", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
