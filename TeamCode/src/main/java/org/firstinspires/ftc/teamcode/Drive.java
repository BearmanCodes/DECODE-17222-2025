package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Drive extends LinearOpMode {
    //private DrivetrainCore dtCore = new DrivetrainCore();
    private CRServo lServo, rServo;

    public ServoImplEx la;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double lPower = 0.1;
    public static double rPower = 0.1;

    public static double lServoPower = 1.0;

    public static double rServoPower = 1.0;

    public static double laPos = 0;


    private DcMotorEx fly, fry;

    @Override
    public void runOpMode() throws InterruptedException {
        //dtCore.init(hardwareMap);
        lServo = hardwareMap.get(CRServo.class, "crL");
        rServo = hardwareMap.get(CRServo.class, "crR");
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fry = hardwareMap.get(DcMotorEx.class, "fry");
        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        la = hardwareMap.get(ServoImplEx.class, "la");

        la.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        la.setPwmEnable();
        la.setPosition(0);
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
               // dtCore.run(gamepad1);
                //double mPower = gamepad1.left_stick_y;
                //lPower = gamepad1.left_trigger;
                //rPower = gamepad1.right_trigger;
                fly.setVelocity(lPower);
                fry.setVelocity(rPower);
                lServo.setPower(lServoPower);
                rServo.setPower(rServoPower);
                la.setPosition(laPos);
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

            }
        }
    }
}
