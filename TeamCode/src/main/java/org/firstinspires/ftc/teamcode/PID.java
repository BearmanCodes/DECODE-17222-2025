package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class PID extends LinearOpMode {
    public DcMotorEx motor;
    public static double gamepadReducer = 0.25;
    public static boolean fwd = false;
    public static double powerCapper = 0.000257;
    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double errTol = 2;
    public final double tpR = 1425.1;
    public double lasterror = 0;
    public double totalErrors = 0;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //3895.9 ticks per revolution. Meaning 360 meaning 2pi radians.
    //Motor currently moves cc given positive power because screw you. Means that positive power go cc and negative go cw. why it's target - curr.

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        //while (!isStopRequested()){

       // }
        controller(tpR);
    }

    public void Init(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (fwd) motor.setDirection(DcMotorSimple.Direction.FORWARD);
        else motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double angular_to_ticks(double angular, boolean is_radian){
        return is_radian ? (tpR / (2 * Math.PI)) * angular : (tpR / 360) * angular;
    }

    public void controller(double target){
        timer.reset();
        double error = target - motor.getCurrentPosition();
        lasterror = error;
        double time = timer.seconds();
        double P = kP * (error);
        double D = kD * ((lasterror - error)/error) * time;
        totalErrors += error * time;
        double I = kI * totalErrors;
        double power = Math.min(Math.max(P + D + I, 0), 1);
        while (error >= errTol && !isStopRequested()) {
            lasterror = error;
            error = target - motor.getCurrentPosition();
            P = kP * (error);
            D = kD * ((lasterror - error)/error) * timer.seconds();
            totalErrors += error * time;
            I = kI * totalErrors;
            power = Math.min(Math.max(P + D + I, 0), 1);
            motor.setPower(power);
            telemetry.addData("P: ", P);
            telemetry.addData("D: ", D);
            telemetry.addData("I: ", I);
            telemetry.addData("Power: ", power);
            telemetry.addData("Error: ", error);
            telemetry.addData("LastError: ", lasterror);
            telemetry.addData("TotalError: ", totalErrors);
            telemetry.addData("Pos: ", motor.getCurrentPosition());
            telemetry.update();
            timer.reset();
        }
        motor.setPower(0);
        telemetry.addData("P: ", P);
        telemetry.addData("D: ", D);
        telemetry.addData("Power: ", power);
        telemetry.addData("Error: ", error);
        telemetry.addData("LastError: ", lasterror);
        telemetry.addData("Pos: ", motor.getCurrentPosition());
        telemetry.addData("FINISHED: ", true);
        telemetry.update();
    }
}
