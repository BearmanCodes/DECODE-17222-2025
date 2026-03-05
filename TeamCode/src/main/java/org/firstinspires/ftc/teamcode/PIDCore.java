package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PIDCore {
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public VoltageSensor voltageSensor;
    public final double MAX_RPM = 6000.0;
    public final double TPR = 28.0;

    public final double RHINO_DIAMETER = 96.0; //mm

    public final double RHINO_RADIUS = 48.0; //mm

    public double previous_error = 0.0;

    public double previous_time = 0.0;

    public static double DESIRED_RPM = 1150;

    double kP;

    double kD;

    double kV;

    //public static double kV = 0.000197633; //2200 RPM LEFT MOTOR

    //public static double kV = 0.000187653; //2200 RPM RIGHT MOTOR

    //public static double kV = 1.01; //works better for RPM of 1875 or vel of 875

    //public static double kP = 0.000935; //pretty good as what it was last time, 0.000935 as Left Motor

    //public static double kP = 0.000935;

    //public static double kD = 0.0075; //pretty good for left side, we'll take it

    public static boolean doThing = true;

    DcMotorEx fly;

    Telemetry telemetry;

    double motorPower;

    public PIDCore(DcMotorEx motor, double kP, double kD, double kV, VoltageSensor voltageSensor, Telemetry telemetry){
        this.voltageSensor = voltageSensor;
        this.telemetry = telemetry;
        this.kP = kP;
        this.kD = kD;
        this.kV = kV;
        this.fly = motor;
    }

    public double PID_calc(double desired_rpm){
        double current_time = timer.time();
        //double DESIRED_POWER = desired_rpm / MAX_RPM;
       // double curr_voltage = voltageSensor.getVoltage();
        double curr_velocity = this.fly.getVelocity();
        double curr_rpm = (curr_velocity / TPR) * 60;
        double feedforward_power = desired_rpm * this.kV;
        double current_error = desired_rpm - curr_rpm;
        double MOTOR_RPM = (curr_velocity / TPR) * 60;
        double WHEEL_MPS = ((2 * Math.PI * RHINO_RADIUS * MOTOR_RPM) / 1000) / 60;
        double P = this.kP * current_error;
        double D = this.kD * (current_error - previous_error) / (current_time - previous_time);
        double output = P + D + feedforward_power;
        //telemetry.addData("Velocity: ", curr_velocity);
      //  telemetry.addData("MOTOR_RPM: ", MOTOR_RPM);
      //  telemetry.addData("DESIRED_RPM: ", desired_rpm);
      //  telemetry.addData("Voltage: ", curr_voltage);
      //  telemetry.addData("WHEEL_MPS: ", WHEEL_MPS);
      //  telemetry.addData("P: ", P);
      //  telemetry.addData("D: ", D);
     //   telemetry.addData("current_error: ", current_error);
     //   telemetry.addData("previous _error: ", previous_error);
     //   telemetry.addData("current_time: ", current_time);
      //  telemetry.addData("previous_time: ", previous_time);
        previous_error = current_error;
        previous_time = current_time;
        return output;
    }
}
