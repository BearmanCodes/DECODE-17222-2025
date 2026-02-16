package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDCore {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public DcMotorEx fly;
    public VoltageSensor voltageSensor;
    public final double MAX_RPM = 6000.0;
    public final double TPR = 28.0;

    public final double RHINO_DIAMETER = 72.0; //mm

    public final double RHINO_RADIUS = 36.0; //mm

    double previous_error = 0.0;

    double previous_time = 0.0;

    public static double DESIRED_RPM = 1875;

    //public static double kV = 1.0783125; //can tune this more but this works alright. Perfect for 4500 RPM, not so much for others, think should tune kV based on desired speeds.

    public static double kV = 1.01; //works better for RPM of 1875 or vel of 875

    public static double kP = 0.000835;

    public static double kD = 0.00431;

    public static boolean doThing = true;

    double motorPower;

    public double PID_calc(DcMotorEx fly, double desired_rpm){
        double current_time = timer.time();
        double DESIRED_POWER = desired_rpm / MAX_RPM;
        double curr_voltage = voltageSensor.getVoltage();
        double curr_velocity = fly.getVelocity();
        double curr_rpm = (curr_velocity / TPR) * 60;
        double feedforward_power = ((DESIRED_POWER * 12.0) / curr_voltage) * kV;
        double current_error = desired_rpm - curr_rpm;
        double MOTOR_RPM = (curr_velocity / TPR) * 60;
        double WHEEL_MPS = ((2 * Math.PI * RHINO_RADIUS * MOTOR_RPM) / 1000) / 60;
        double P = kP * current_error;
        double D = kD * (current_error - previous_error) / (current_time - previous_time);
        double output = P + D + feedforward_power;
        //telemetry.addData("Velocity: ", curr_velocity);
        //telemetry.addData("MOTOR_RPM: ", MOTOR_RPM);
        //telemetry.addData("DESIRED_RPM: ", desired_rpm);
        //telemetry.addData("Voltage: ", curr_voltage);
        //telemetry.addData("WHEEL_MPS: ", WHEEL_MPS);
        //telemetry.addData("P: ", P);
        //telemetry.addData("D: ", D);
        //telemetry.addData("current_error: ", current_error);
        //telemetry.addData("previous _error: ", previous_error);
        //telemetry.addData("current_time: ", current_time);
        //telemetry.addData("previous_time: ", previous_time);
        previous_error = current_error;
        previous_time = current_time;
        return output;
    }
}
