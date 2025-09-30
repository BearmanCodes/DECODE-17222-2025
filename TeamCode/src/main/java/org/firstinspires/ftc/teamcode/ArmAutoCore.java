package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmAutoCore {
    public DcMotorEx pvtArm;
    public static int errTol = 5;
    public static int err;
    public static int ticks;
    public static int armCurrPos;
    public static boolean running;

    public void init(HardwareMap hwMap){
        pvtArm = hwMap.get(DcMotorEx.class, "pvt");

        pvtArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        pvtArm.setDirection(DcMotorSimple.Direction.REVERSE);
        pvtArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pvtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pvtMove(double velocity, int ticksVar, boolean active, Telemetry tele)  {
        armCurrPos = pvtArm.getCurrentPosition();
        ticks = ticksVar;
        pvtArm.setTargetPosition(ticks);
        pvtArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pvtArm.setVelocity(velocity);
        err = Math.abs(armCurrPos - ticks); //amount of ticks to go to target
        running = true;
        if (err <= errTol){
            pvtArm.setVelocity(0);
            pvtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            running = false;
        }

    }

    public void checkArm(){
        if (running){
            armCurrPos = pvtArm.getCurrentPosition();
            err = Math.abs(armCurrPos - ticks); //amount of ticks to go to target
            if (err <= errTol){
                pvtArm.setVelocity(0);
                pvtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                running = false;
            }
        }
    }
}
