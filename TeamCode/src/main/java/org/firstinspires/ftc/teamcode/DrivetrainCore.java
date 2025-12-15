package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.BufferedWriter;
import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


//This is the core drivetrain class that every TeleOp uses when they need to access drivetrain features.

//NOTE, IF THERE'S A PROBLEM WITH MOTOR(S) FASTER THAN THE OTHER(S) CHECK THE ENCODER CABLE
public class DrivetrainCore{

    public static DcMotorEx frontleft;
    public static DcMotorEx frontright;
    public static DcMotorEx backleft;
    public static DcMotorEx backright; //Declare the drivetrian motors
    public static ServoImplEx la;
    public static double reducer = 1; //Change for reducing drive power
    YawPitchRollAngles robotOrientation; //IMU YPR Angles
    public List<Integer> valid_ids = Arrays.asList(21, 22, 23, 20, 24);

    private AprilTagProcessor aTag;

    private VisionPortal visionPortal;

    static IMU imu; //Declare the IMU
    static IMU.Parameters imuparams; //Declare the IMU's settingsx

    public void init(HardwareMap hwMap) throws IOException {
        frontleft = hwMap.get(DcMotorEx.class, "frontleft");  //change these motor names depending on the config
        frontright = hwMap.get(DcMotorEx.class, "frontright");
        backleft = hwMap.get(DcMotorEx.class, "backleft");
        backright = hwMap.get(DcMotorEx.class, "backright");

        motorSetUp(); //Assigns all needed motor modes and settings out of view
    }


    public void run(Gamepad gamepad1){ //Main running function, TeleOp's will use this function.
        double Vertical = gamepad1.left_stick_y;
        double Horizontal = gamepad1.left_stick_x;
        double Pivot = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(Vertical) + Math.abs(Horizontal) + Math.abs(Pivot), 1);

        double frontLeftPower = (-Pivot + (Vertical - Horizontal)) * reducer;
        double frontRightPower = (Pivot + Vertical + Horizontal) * reducer; //Mecanum drivetrain shenanigans
        double backRightPower = (Pivot + (Vertical - Horizontal)) * reducer;
        double backLeftPower = (-Pivot + Vertical + Horizontal) * reducer;

        //I don't understand any of this math but it allows the mecanum wheels to, do what they do.
        frontleft.setPower(frontLeftPower);
        frontright.setPower(frontRightPower);
        backleft.setPower(backLeftPower);
        backright.setPower(backRightPower);
    }

    public void InitFile(){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("yourmom.csv"))) {
            bw.write("X,Y,Z,Pitch,Roll,Yaw,Range,Bearing,Elevation,Power,LA Pos");
            System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
            System.out.println("Error writing file.");
        }
    }

    public void AppendData(){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("yourmom.csv"))) {
            bw.newLine();
            bw.write("X,Y,Z,Pitch,Roll,Yaw,Range,Bearing,Elevation,Power,LA Pos");
            System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
            System.out.println("Error writing file.");
        }
    }

    public void allMotorPower(double power){
        frontright.setPower(power);
        frontleft.setPower(power);
        backleft.setPower(power);
        backright.setPower(power);
    }

    public static void motorSetUp(){ //Background work for declaring motor modes and settings
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD); //Change these directions for your drive
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //below are functions of convenience that apply to all the motors in just one line of code

    public void setMotorVelocity(double fLvelocity, double fRvelocity, double bLvelocity, double bRvelocity) {
        frontleft.setVelocity(fLvelocity);
        frontright.setVelocity(fRvelocity);
        backleft.setVelocity(bLvelocity);
        backright.setVelocity(bRvelocity);
    }

    public void setMotorPower(double fLvelocity, double fRvelocity, double bLvelocity, double bRvelocity) {
        frontleft.setPower(fLvelocity);
        frontright.setPower(fRvelocity);
        backleft.setPower(bLvelocity);
        backright.setPower(bRvelocity);
    }

    public void allMotorVelocity(double velocity) {
        frontleft.setVelocity(velocity);
        frontright.setVelocity(velocity);
        backleft.setVelocity(velocity);
        backright.setVelocity(velocity);
    }

}