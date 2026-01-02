package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ShooterAutoCore {
    public ServoImplEx laR, laL;
    public CRServo lServo, rServo;

    public static double flyExpected, fryExpected;

    public DcMotorEx fly, fry;

    public static double laInitPos = 0.25;


    public void init(HardwareMap hwMap){
        lServo = hwMap.get(CRServo .class, "crl");
        rServo = hwMap.get(CRServo.class, "crr");
        fly = hwMap.get(DcMotorEx .class, "fly");
        fry = hwMap.get(DcMotorEx.class, "fry");

        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }

    //spinupFlys(1100)
        //setLauncherPos(0.42)
            //setCRPower(1)

    public void power_surge(double surge_measure){
        boolean leftMotorSurge = (fly.getVelocity() - load_fly_expected()) >= surge_measure;
        boolean rightMotorSurge = (fry.getVelocity() - load_fry_expected()) >= surge_measure;
        boolean leftMotorRunning = load_fry_expected() >= 200;
        boolean rightMotorRunning = load_fry_expected() >= 200;
        boolean leftServoRunning = lServo.getPower() > 0;
        boolean rightServoRunning = rServo.getPower() > 0;
        boolean EVERYTHING = leftMotorSurge && rightMotorSurge && leftMotorRunning && rightMotorRunning && leftServoRunning && rightServoRunning;
        if (EVERYTHING){
            setCRPower(0);
        }
    }

    public double load_fly_expected(){
        return flyExpected;
    }

    public double load_fry_expected(){
        return fryExpected;
    }

    public void setCRPower(double power){
        lServo.setPower(power);
        rServo.setPower(power);
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
}
