package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.PIDCore;

@TeleOp
@Config
public class PIDTester extends OpMode {
    public static double L_RPM = 2200;

    public static double R_RPM = 2200;

    public static double L_kP = 0.000935;
    public static double L_kD = 0.0075;

    public static double L_kV = 0.000197633;

    public static double R_kP = 0.000935;

    public static double R_kD = 0.0075;

    public static double R_kV = 0.000187653;

    public final double TPR = 28.0;

    PIDCore pidCore;
    VoltageSensor voltageSensor;

    PIDCore flyController;

    PIDCore fryController;

    public DcMotorEx fly, fry;


    @Override
    public void init() {
        fry = hardwareMap.get(DcMotorEx.class, "fry");
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fly.setVelocityPIDFCoefficients(LP, LI, LD, LF);
        //fry.setVelocityPIDFCoefficients(RP, RI, RD, RF);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flyController = new PIDCore(fly, L_kP, L_kD, L_kV, voltageSensor, telemetry);
        fryController = new PIDCore(fry, R_kP, R_kD, R_kV, voltageSensor, telemetry);
    }

    @Override
    public void loop() {
        fly.setPower(flyController.PID_calc(L_RPM));
        fry.setPower(fryController.PID_calc(R_RPM));
        telemetry.addData("Vel: ", fly.getVelocity());
        telemetry.addData("Ver: ", fry.getVelocity());
        telemetry.addData("Desired Vel: ", (L_RPM / 60) * 28);
        telemetry.addData("Desired Ver: ", (R_RPM / 60) * 28);
        telemetry.addData("LPM: ", (fly.getVelocity() / TPR) * 60);
        telemetry.addData("RPM: ", (fry.getVelocity() / TPR) * 60);
        telemetry.addData("Desired LPM: ", L_RPM);
        telemetry.addData("Desired RPM: ", R_RPM);
        telemetry.update();
    }
}
