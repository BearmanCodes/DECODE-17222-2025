package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
public class ShooterAutoCore {
    public ServoImplEx laR, laL;

    public Servo luigiServo;

    public static double KICK_ITERATOR = 0.325;

    public static double FAILSAFE_WAIT = 3500;

    public long failsafeTime;

    public boolean canAddShot = true;

    public static double luigiBlock = 0.47;

    long entry_time = 0;

    public static double luigiFlow = 0.15;

    public static int SHOOT_INTERMITENT_TIME_MS = 1500;

    public static int SHOOT_INTAKE_TIME_MS = 750;

    public CRServo lServo, rServo;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public int shotsTaken = 0;

    public static int SURGE_MEASURE = 150;

    public static int RUNNING_MODIFIER = 350;

    public boolean hasSurged = false;

    public static DcMotorEx intake;

    public static double reducer = 1;

    public static double flyExpected, fryExpected;

    public DcMotorEx fly, fry;

    public static double laInitPos = 0.53;


    public void init(HardwareMap hwMap){
        lServo = hwMap.get(CRServo .class, "crl");
        rServo = hwMap.get(CRServo.class, "crr");
        fly = hwMap.get(DcMotorEx .class, "fly");
        fry = hwMap.get(DcMotorEx.class, "fry");

        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);

        luigiServo = hwMap.get(Servo.class, "WEARE");
        luigiServo.setPosition(luigiFlow); //0.19 is back enough that balls fall, //0.52 is straight block

        laR = hwMap.get(ServoImplEx.class, "lar");

        laR.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        laR.setPwmEnable();
        laR.setPosition(laInitPos);

        laL = hwMap.get(ServoImplEx.class, "lal");

        laL.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        laL.setPwmEnable();
        laL.setPosition(laInitPos);

        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //spinupFlys(1100)
        //setLauncherPos(0.42)
            //setCRPower(1)

    public void in(){
        intake.setPower(reducer);
    }

    public void stop(){
        intake.setPower(0);
    }

    public void out(){
        intake.setPower(-reducer);
    }

    public boolean power_surge(double surge_measure, Telemetry tele){
        boolean leftMotorSurge = (load_fly_expected() - fly.getVelocity()) >= surge_measure;
        boolean rightMotorSurge = (load_fry_expected() - fry.getVelocity()) >= surge_measure;
        boolean leftMotorRunning = fly.getVelocity() >= RUNNING_MODIFIER;
        boolean rightMotorRunning = fry.getVelocity() >= RUNNING_MODIFIER;
        boolean leftServoRunning = lServo.getPower() > 0;
        boolean rightServoRunning = rServo.getPower() > 0;
        boolean EVERYTHING = leftMotorSurge && rightMotorSurge && leftMotorRunning && rightMotorRunning && leftServoRunning && rightServoRunning;
        tele.addData("leftMotorSurge: ", leftMotorSurge);
        tele.addData("fly - actual: ", load_fly_expected() - fly.getVelocity());
        tele.addData("rightMotorSurge: ", rightMotorSurge);
        tele.addData("fry - actual: ", load_fry_expected() - fry.getVelocity());
        tele.addData("leftMotorRunning: ", leftMotorRunning);
        tele.addData("Fly Vel: ", fly.getVelocity());
        tele.addData("Fly Cutoff_Vel: ", load_fly_expected() - (surge_measure - RUNNING_MODIFIER));
        tele.addData("rightMotorRunning: ", rightMotorRunning);
        tele.addData("Fry Vel: ", fry.getVelocity());
        tele.addData("Fry Cutoff_Vel: ", load_fry_expected() - (surge_measure - RUNNING_MODIFIER));
        tele.addData("leftServoRunning: ", leftServoRunning);
        tele.addData("rightServoRunning: ", rightServoRunning);
        tele.addData("EVERYTHING: ", EVERYTHING);
        tele.update();
        return EVERYTHING;
    }

    public double load_fly_expected(){
        return flyExpected;
    }

    public double load_fry_expected(){
        return fryExpected;
    }

    public void setCRPower(double power, Telemetry telemetry){
        lServo.setPower(power);
        rServo.setPower(power);
        telemetry.addData("Power should be: ", power);
        telemetry.update();
    }

    public void spinUpFlys(double lVel, double rVel){
        fly.setVelocity(lVel);
        flyExpected = lVel;
        fry.setVelocity(rVel);
        fryExpected = rVel;
    }

    public void setLauncherPos(double pos){
        laR.setPosition(pos);
        laL.setPosition(pos);
    }

    public boolean shoot(int shots, Telemetry tele){
        if (shotsTaken < shots){
            if (power_surge(SURGE_MEASURE, tele)){
                tele.update();
                entry_time = timer.now(TimeUnit.MILLISECONDS);
                setCRPower(-1, tele);
                luigiServo.setPosition(luigiFlow);
                stop();
                if (canAddShot){
                    shotsTaken++;
                    canAddShot = false;
                }
            }
            if (entry_time > 0 && timer.now(TimeUnit.MILLISECONDS) - entry_time >= SHOOT_INTERMITENT_TIME_MS){
                entry_time = 0;
                //in();
                setCRPower(1, tele);
                luigiServo.setPosition(luigiBlock);
                canAddShot = true;
                tele.addData("THIS WAS ENTERED: ", true);
                tele.update();
            }
            tele.addData("Shots: ", shotsTaken);
            tele.addData("entry_time: ", entry_time);
            tele.addData("time right now: ", timer.now(TimeUnit.MILLISECONDS));
            tele.update();
            return false;
        } else {
            stop();
            setCRPower(0, tele);
            shotsTaken = 0;
            canAddShot = true;
            tele.update();
            tele.addData("DONE: ", true);
            return true;
        }
    }

    public boolean intakeShoot(int shots, Telemetry tele){
        luigiServo.setPosition(luigiBlock);
        if (shotsTaken < shots){
            if (power_surge(SURGE_MEASURE, tele)){
                entry_time = timer.now(TimeUnit.MILLISECONDS);
                setCRPower(0, tele);
                if (canAddShot){
                    shotsTaken++;
                    canAddShot = false;
                }
            }
            if (entry_time > 0 && timer.now(TimeUnit.MILLISECONDS) - entry_time >= SHOOT_INTAKE_TIME_MS){
                entry_time = 0;
                setCRPower(1, tele);
                canAddShot = true;
                tele.addData("THIS WAS ENTERED: ", true);
            }
            tele.addData("Shots: ", shotsTaken);
            tele.addData("entry_time: ", entry_time);
            tele.addData("time right now: ", timer.now(TimeUnit.MILLISECONDS));
            tele.update();
            return false;
        } else {
            stop();
            setCRPower(0, tele);
            tele.addData("DONE: ", true);
            return true;
        }
    }
}
