package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Laser;
@TeleOp
public class LaserTeesting extends OpMode {
    Laser laserSensor;
    public static boolean INTAKE_RUN_FWD = false;

    public static boolean INTAKE_RUN_REV = false;

    public static double inPower;

    DcMotorEx intake;
    private int ballCount = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        laserSensor = new Laser(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (laserSensor.isBallDetected()) {
            ballCount += 1;
        }
        telemetry.addData("Ball Count: ", ballCount);
        telemetry.update();
        intake.setPower(inPower);
        intakeControls();
    }

    private void intakeControls(){
        if (gamepad1.leftBumperWasPressed()) {
            INTAKE_RUN_FWD = !INTAKE_RUN_FWD;
            INTAKE_RUN_REV = false;
            if (INTAKE_RUN_FWD) {
                inPower = 0.75;
            } else {
                inPower = 0;
            }
        }
        if (gamepad1.rightBumperWasPressed()) {
            INTAKE_RUN_REV = !INTAKE_RUN_REV;
            INTAKE_RUN_FWD = false;
            if (INTAKE_RUN_REV) {
                inPower = -0.75;
            } else {
                inPower = 0;
            }
        }
        intake.setPower(inPower);
    }
}
