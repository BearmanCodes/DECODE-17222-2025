package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public CRServo lServo, rServo;

    public DcMotorEx fly, fry;
    public double L_RPM = 0;
    public double R_RPM = 0;
    public double flyExpectedVel, fryExpectedVel;
    public double SURGE_MEASURE = 250;
    public double RUNNING_SPEEDS = 350;


    public Servo luigiServo;

    public ServoImplEx laL, laR;
    public double laInitPos = 0.32;
    public PIDCore pidCore;
    public VoltageSensor voltageSensor;
    public Telemetry telemetry;

    public boolean isShooting = false;
    public boolean hasSurged = false;
    public long entry_time = 0;
    ElapsedTime shot_timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double SHOOT_INTERMITENT_TIME_MS = 600;
    public double luigiKick = ModeCore.LUIGI_HOPPER_SHOOT;
    public double luigiLoad = ModeCore.LUIGI_HOPPER_LOAD;

    public OpShooterCore(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hwMap) {

        rServo = hwMap.get(CRServo.class, "crr");
        lServo = hwMap.get(CRServo.class, "crl");
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);

        fry = hwMap.get(DcMotorEx.class, "fry");
        fly = hwMap.get(DcMotorEx.class, "fly");
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hwMap.voltageSensor.iterator().next();
        pidCore = new PIDCore(voltageSensor, telemetry);

        luigiServo = hwMap.get(Servo.class, "WEARE");

        laR = hwMap.get(ServoImplEx.class, "lar");
        laR.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        laR.setPwmEnable();
        //laR.setPosition(laInitPos);
        laL = hwMap.get(ServoImplEx.class, "lal");
        laL.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        laL.setPwmEnable();
        //laL.setPosition(laInitPos);
    }
    public void FlysPIDControl(){
        fly.setPower(pidCore.PID_calc(fly, L_RPM));
        fry.setPower(pidCore.PID_calc(fry, R_RPM));
    }

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
