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

    public static int L_VEL = 900;

    public static int R_VEL = 900;

    Telemetry dashTele = dashboard.getTelemetry();

    ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

    @Override
    public void runOpMode() throws InterruptedException {
        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
        waitForStart();
        shooterAutoCore.in();
        //shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow + ShooterAutoCore.KICK_ITERATOR);
        shooterAutoCore.setCRPower(1, dashTele);
        while (!isStopRequested()){
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            shooterAutoCore.setLauncherPos(ShooterAutoCore.laInitPos);
            shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
            shooterAutoCore.intakeShoot(3, dashTele);
        }
    }
}
