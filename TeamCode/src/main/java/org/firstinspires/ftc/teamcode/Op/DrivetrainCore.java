package org.firstinspires.ftc.teamcode.Op;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainCore{
    static DcMotorSimple.Direction FL_DIR = DcMotorSimple.Direction.REVERSE;
    static DcMotorSimple.Direction FR_DIR = DcMotorSimple.Direction.FORWARD;
    static DcMotorSimple.Direction BL_DIR = DcMotorSimple.Direction.REVERSE;
    static DcMotorSimple.Direction BR_DIR = DcMotorSimple.Direction.FORWARD;

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    public void Init(HardwareMap hwmap){
        frontLeft = hwmap.get(DcMotorEx.class, "frontleft");
        frontRight = hwmap.get(DcMotorEx.class, "frontright");
        backLeft = hwmap.get(DcMotorEx.class, "backleft");
        backRight = hwmap.get(DcMotorEx.class, "backright");
        frontLeft.setDirection(FL_DIR);
        frontRight.setDirection(FR_DIR);
        backLeft.setDirection(BL_DIR);
        backRight.setDirection(BR_DIR);
    }

    public void setDrivetrainPower(double flPower, double frPower, double blPower, double brPower){
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }
}
