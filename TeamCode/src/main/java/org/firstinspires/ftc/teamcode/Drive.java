package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Drive extends LinearOpMode {
    //private DrivetrainCore dtCore = new DrivetrainCore();
    private CRServo servo;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double lPower = 0.1;
    public static double rPower = 0.1;


    private DcMotorEx fly, fry;

    @Override
    public void runOpMode() throws InterruptedException {
        //dtCore.init(hardwareMap);
        servo = hardwareMap.get(CRServo.class, "cr");
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fry = hardwareMap.get(DcMotorEx.class, "fry");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
               // dtCore.run(gamepad1);
                //double mPower = gamepad1.left_stick_y;
                //lPower = gamepad1.left_trigger;
                //rPower = gamepad1.right_trigger;
                fly.setVelocity(lPower);
                fry.setVelocity(rPower);
                telemetry.addData("L Power: ", lPower);
                telemetry.addData("L RPM: ", 6000 * lPower);
                telemetry.addData("L Output Velocity: ", fly.getVelocity());

                telemetry.addData("R Power: ", rPower);
                telemetry.addData("R RPM: ", 6000 * rPower);
                telemetry.addData("R Output Velocity: ", fry.getVelocity());
                telemetry.update();

                dashTele.addData("L Power: ", lPower);
                dashTele.addData("L RPM: ", 6000 * lPower);
                dashTele.addData("L Output Velocity: ", fly.getVelocity());
                dashTele.addData("time passed: ", timer.now(TimeUnit.MILLISECONDS));

                dashTele.addData("R Power: ", rPower);
                dashTele.addData("R RPM: ", 6000 * rPower);
                dashTele.addData("R Output Velocity: ", fry.getVelocity());
                dashTele.update();
                if (gamepad1.x && !gamepad1.xWasPressed()){
                    servo.setPower(1);
                    telemetry.addLine("Should be going");
                    telemetry.update();
                }

                if (gamepad1.y && !gamepad1.yWasPressed()){
                    servo.setPower(-1);
                    telemetry.addLine("Should be going back");
                    telemetry.update();
                }

                if (gamepad1.b && !gamepad1.bWasPressed()){
                    servo.setPower(0);
                    telemetry.addLine("Should be going back");
                    telemetry.update();
                }

            }
        }
    }
}
