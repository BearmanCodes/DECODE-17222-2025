package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class ShootingTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int L_VEL, R_VEL = 850;

    Telemetry dashTele = dashboard.getTelemetry();


    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    @Override
    public void runOpMode() throws InterruptedException {
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        waitForStart();
        shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow + ShooterAutoCore.KICK_ITERATOR);
        shooterAutoCore.setCRPower(1, dashTele);
        //shooterAutoCore.in();
        while (!isStopRequested()){
            shooterAutoCore.shoot(3, dashTele);
        }
    }
}
