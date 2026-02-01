package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Temporary.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class CleaningMode extends LinearOpMode {


    public static int L_VEL = 150;
    //RED 1000

    public static int R_VEL = 150;
    //RED 1225

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    public static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double LUIGI_HOPPER_LOAD = 0.065;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(LUIGI_HOPPER_LOAD);
        waitForStart();
        while (!isStopRequested()){
            if (gamepad1.leftStickButtonWasPressed()) {
                shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
                shooterAutoCore.setCRPower(0, telemetry);
            }
            if (gamepad1.rightStickButtonWasPressed()) {
                shooterAutoCore.spinUpFlys(0, 0);
                shooterAutoCore.setCRPower(1, telemetry);
            }
        }
    }
}
