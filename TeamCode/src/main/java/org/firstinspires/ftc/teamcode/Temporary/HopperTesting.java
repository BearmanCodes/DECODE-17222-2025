package org.firstinspires.ftc.teamcode.Temporary;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class HopperTesting extends LinearOpMode {

    public static Follower follower;

    public static int STOPPED_POWER = -1;


    public static int L_VEL = 900;
    //RED 1000

    public static int R_VEL = 975;
    //RED 1225

    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180));
                                            //BLUE X: 88.08 Y: 13.61 H: 71.02 DEG LAUNCHER POS: .32. RED X:  Y: H: 35.5 DEG LAUNCHER POS:

    public static Pose startingPose;

    public static boolean useDefaultPose = true;

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    public static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static boolean hasShot = false;

    public static double SURGE_TOLERANCE = 150;

    public static double KICK_SERVO_POS = 0.35;

    public static double MEET_SPEED_TOL = 15;

    public static double LUIGI_HOPPER_LOAD = 0.065;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(LUIGI_HOPPER_LOAD);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        follower.update();
        waitForStart();
        follower.startTeleopDrive(true);
        while (!isStopRequested()){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.update();
            telemetry.update();
            shooterAutoCore.updateLaPos(gamepad1, telemetry);
            shooterAutoCore.updateLuigiBlock(gamepad1, telemetry);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            if (shooterAutoCore.load_fly_expected() - shooterAutoCore.fly.getVelocity() >= SURGE_TOLERANCE && shooterAutoCore.load_fry_expected() - shooterAutoCore.fry.getVelocity() >= SURGE_TOLERANCE) {
                timer.reset();
                hasShot = true;
            }
            if (hasShot) {
                if (shooterAutoCore.load_fly_expected() - shooterAutoCore.fly.getVelocity() <= MEET_SPEED_TOL && shooterAutoCore.load_fry_expected() - shooterAutoCore.fry.getVelocity() <= MEET_SPEED_TOL){
                    telemetry.addData("Time: ", timer.time(TimeUnit.MILLISECONDS));
                    hasShot = false;
                    timer.reset();
                }
            }
            if (gamepad1.crossWasPressed()){
                shooterAutoCore.setCRPower(1, telemetry);
                shooterAutoCore.luigiServo.setPosition(KICK_SERVO_POS);
            }
            if (gamepad1.triangleWasPressed()) {
                shooterAutoCore.setCRPower(STOPPED_POWER, telemetry);
                shooterAutoCore.luigiServo.setPosition(0.065);
            }
            dualTele("Follower X: ", follower.getPose().getX());
            dualTele("Follower Y: ", follower.getPose().getY());
            dualTele("Follower Heading (rad): ", follower.getPose().getHeading());
            dualTele("Follower Heading (deg): ", Math.toDegrees(follower.getPose().getHeading()));
            dualTele("FLY_EXPECTED_VEL: ", shooterAutoCore.load_fly_expected());
            dualTele("FRY_EXPECTED_VEL: ", shooterAutoCore.load_fry_expected());
            dualTele("FLY_REAL_VEL: ", shooterAutoCore.fly.getVelocity());
            dualTele("FRY_REAL_VEL: ", shooterAutoCore.fry.getVelocity());
        }
    }
    public void dualTele(String caption, Object data){
        telemetry.addData(caption, data);
    }
}
