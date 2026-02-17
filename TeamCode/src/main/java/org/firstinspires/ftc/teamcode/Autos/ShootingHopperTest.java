package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class ShootingHopperTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int L_VEL = 1100;

    public static int R_VEL = 1100;

    Telemetry dashTele = dashboard.getTelemetry();

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore(telemetry);

    public static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static boolean hasShot = false;

    public static double SURGE_TOLERANCE = 150;

    public static double MEET_SPEED_TOL = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterAutoCore.init(hardwareMap);

        shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        waitForStart();

        while (!isStopRequested()){
            shooterAutoCore.updateLaPos(gamepad1, dashTele);
            shooterAutoCore.updateLuigiBlock(gamepad1, dashTele);
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            if (shooterAutoCore.load_fly_expected() - shooterAutoCore.fly.getVelocity() >= SURGE_TOLERANCE && shooterAutoCore.load_fry_expected() - shooterAutoCore.fry.getVelocity() >= SURGE_TOLERANCE) {
                timer.reset();
                hasShot = true;
            }
            if (hasShot) {
                if (shooterAutoCore.load_fly_expected() - shooterAutoCore.fly.getVelocity() <= MEET_SPEED_TOL && shooterAutoCore.load_fry_expected() - shooterAutoCore.fry.getVelocity() <= MEET_SPEED_TOL){
                    dashTele.addData("Time: ", timer.time(TimeUnit.MILLISECONDS));
                    dashTele.update();
                    hasShot = false;
                    timer.reset();
                }
            }
            if (gamepad1.xWasPressed()){
                shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
                shooterAutoCore.setCRPower(1, dashTele);
                while (!isStopRequested() && !shooterAutoCore.shoot(3, dashTele)) {
                    shooterAutoCore.updateLaPos(gamepad1, dashTele);
                    shooterAutoCore.updateLuigiBlock(gamepad1, dashTele);
                    dashTele.update();
                }
            }
        }
    }
}
