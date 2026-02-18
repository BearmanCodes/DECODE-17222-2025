package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDCore;

import java.util.concurrent.TimeUnit;

@Config
public class OpShooterCore {
    public CRServo lServo, rServo, boot;

    public DcMotorEx fly, fry;

    double green_count = 0;

    double purple_count = 0;

    boolean currentlyGreen = false;
    boolean previouslyGreen = false;
    boolean currentlyPurple = false;
    boolean previouslyPurple = false;

    public static double LP = 161;
    public static double colorTolerances = 450;

    public static double specificTolerances = 500;

    public static double LI = 0.5;

    public static double LD = 0.5;

    public static double LF = 12.5;

    public static double RP = 195;

    public static double RI = 0.4;

    public static double RD = 0.75;

    public static double RF = 9.5;
    public double L_RPM = 0;
    public double R_RPM = 0;
    public double flyExpectedVel, fryExpectedVel;
    public double SURGE_MEASURE = 150;
    public static double RUNNING_SPEEDS = 350;

    public Servo luigiServo;

    public ColorSensor colorSensor;

    public static Servo laL, laR;
    public static double laInitPos = 0;
    public PIDCore pidCore;
    public VoltageSensor voltageSensor;
    public Telemetry telemetry;

    public boolean isShooting = false;
    public boolean hasSurged = false;
    public long entry_time = 0;
    ElapsedTime shot_timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static double SHOOT_INTERMITENT_TIME_MS = 275;
    public double luigiKick = ModeCore.LUIGI_HOPPER_SHOOT;
    public double luigiLoad = ModeCore.LUIGI_HOPPER_LOAD;

    //PIDCore pidCore;

    public OpShooterCore(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hwMap) {

        rServo = hwMap.get(CRServo.class, "crr");
        lServo = hwMap.get(CRServo.class, "crl");
        boot = hwMap.get(CRServo.class, "boot");
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        boot.setDirection(DcMotorSimple.Direction.REVERSE);
        boot.setPower(0);

        fry = hwMap.get(DcMotorEx.class, "fry");
        fly = hwMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fly.setVelocityPIDFCoefficients(LP, LI, LD, LF);
        //fry.setVelocityPIDFCoefficients(RP, RI, RD, RF);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hwMap.voltageSensor.iterator().next();
        pidCore = new PIDCore(voltageSensor, telemetry);
        PIDCore.kV = 1.00768;

        luigiServo = hwMap.get(Servo.class, "WEARE");

        colorSensor = hwMap.get(ColorSensor.class, "color");

        laL = hwMap.get(Servo.class ,"lal");

        laL.setDirection(Servo.Direction.REVERSE);

        laR = hwMap.get(Servo.class, "lar");


        laL.setPosition(laInitPos);
        laR.setPosition(laInitPos);

        //laL.setPosition(laInitPos);
    }
    public void FlysPIDControl(){
        fly.setPower(pidCore.PID_calc(fly, L_RPM));
        fry.setPower(pidCore.PID_calc(fry, R_RPM));
    }

    public void updateColors(){
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();
        boolean isRed = red >= specificTolerances;
        boolean isGreen = green >= specificTolerances;
        boolean isBlue = blue >= specificTolerances;
        boolean greenBall = isGreen && green > red && green > blue;
        boolean purpleBall = (isRed && isBlue) && green < red && green < blue;
        updateColorState(greenBall, purpleBall);
        if (currentlyGreen && !previouslyGreen){
            green_count += 1;
        }
        if (currentlyPurple && !previouslyPurple) {
            purple_count += 1;
        }
        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);
        telemetry.addData("Green Balls: ", green_count);
        telemetry.addData("Purple Balls: ", purple_count);
    }

    public void updateColorState(boolean green, boolean purple){
        previouslyGreen = currentlyGreen;
        currentlyGreen = green;
        previouslyPurple = currentlyPurple;
        currentlyPurple = purple;
    }

    /*
    public void setFlySpeeds(double lRPM, double rRPM){
        fly.setVelocity(lRPM);
        flyExpectedVel = lRPM;
        fry.setVelocity(rRPM);
        fryExpectedVel = rRPM;
    }
    */


    public void setFlySpeeds(double lRPM, double rRPM){
        L_RPM = lRPM;
        flyExpectedVel = (lRPM / 60) * 28;
        R_RPM = rRPM;
        fryExpectedVel = (rRPM / 60) * 28;
    }

    public double load_fly_expected_vel(){
        return flyExpectedVel;
    }
    public double load_fry_expected_vel(){
        return fryExpectedVel;
    }

    public void boot_fwd(){
        boot.setPower(1);
    }

    public void boot_rev(){
        boot.setPower(-1);
    }

    public void boot_stop(){
        boot.setPower(0);
    }

    public boolean power_surge(){
        boolean leftMotorSurge = (load_fly_expected_vel() - fly.getVelocity()) >= SURGE_MEASURE;
        boolean rightMotorSurge = (load_fry_expected_vel() - fry.getVelocity()) >= SURGE_MEASURE;
        boolean leftMotorRunning = fly.getVelocity() >= RUNNING_SPEEDS;
        boolean rightMotorRunning = fry.getVelocity() >= RUNNING_SPEEDS;
        boolean leftServoRunning = lServo.getPower() > 0;
        boolean rightServoRunning = rServo.getPower() > 0;
        boolean EVERYTHING = leftMotorSurge && rightMotorSurge && leftMotorRunning && rightMotorRunning && leftServoRunning && rightServoRunning;
        return EVERYTHING;
    }

    public void setCRPower(double power){
        lServo.setPower(power);
        rServo.setPower(power);
    }

    public void luigiKick(){
        luigiServo.setPosition(luigiKick);
    }

    public void luigiLoad(){
        luigiServo.setPosition(luigiLoad);
    }

    public void start_shoot_once(){
        isShooting = true;
        setCRPower(1);
        luigiKick();
        hasSurged = false;
        entry_time = 0;
    }
    public void stop_shoot_once(){
        isShooting = false;
        setCRPower(-1);
        luigiLoad();
        hasSurged = false;
        entry_time = 0;
    }

    public static void setLauncherPos(double position){
        laL.setPosition(position);
        laR.setPosition(position);
    }

    public void shooting_loop() {
        if (isShooting) {
            if (power_surge()) {
                hasSurged = true;
                entry_time = shot_timer.now(TimeUnit.MILLISECONDS);
                setCRPower(-1);
                luigiLoad();
            }
            if (hasSurged && shot_timer.now(TimeUnit.MILLISECONDS) - entry_time >= SHOOT_INTERMITENT_TIME_MS) {
                hasSurged = false;
                entry_time = 0;
                setCRPower(1);
                luigiKick();
            }
        }
    }
}
