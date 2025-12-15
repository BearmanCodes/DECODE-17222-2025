package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveAutoCore {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    ArmAutoCore armCore = new ArmAutoCore();


    YawPitchRollAngles robotOrientation;
    public IMU imu;
    public IMU.Parameters imuparams;

    static final double TicksPerRev = 537.7;
    static final double WheelInchesCircumference = 2 * Math.PI * (52 / 25.4);
                                                    //12.863214014698366015752555585081
    static final double BotInchesCircumference = 18 * Math.PI;
                                                    //56.548667764616278292327580899031

    static final double WheelRevsPerBotRev = BotInchesCircumference / WheelInchesCircumference;
                                                    //4.396153846153846153846153846154

    static final double TicksPerRadian = WheelRevsPerBotRev / (2 * Math.PI);

    static final double TicksPerIn = TicksPerRev / (WheelInchesCircumference * Math.PI);

    //want to go 590.952980767 ticks



    public void Drive(double velocity,
                      double frontLeftInches, double frontRightInches,
                      double backLeftInches, double backRightInches, boolean active,
                      long timeout) throws InterruptedException {
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;

        // Ensure that the opmode is still active
        if (active) {
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * TicksPerIn);
            frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * TicksPerIn);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * TicksPerIn);
            backRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * TicksPerIn);

            allTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            frontLeft.getCurrentPosition();

            allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            allMotorVelocity(Math.abs(velocity));

            while (active && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            }

            allMotorVelocity(0);

            allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Thread.sleep(timeout);
        }
    }
    public void init(HardwareMap hwMap){
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft".toLowerCase());
        frontRight = hwMap.get(DcMotorEx.class, "frontRight".toLowerCase());
        backLeft = hwMap.get(DcMotorEx.class, "backLeft".toLowerCase());
        backRight = hwMap.get(DcMotorEx.class, "backRight".toLowerCase());


        imu = hwMap.get(IMU.class, "imu");
        imuparams = new IMU.Parameters(new RevHubOrientationOnRobot
                (RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(imuparams);
        imu.resetYaw();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armCore.init(hwMap);
    }

    public void allMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void allMotorVelocity(double velocity) {
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }

    public void allMotorPower(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower,
                              double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void setMotorVelocity(double fLvelocity, double fRvelocity, double bLvelocity, double bRvelocity) {
        frontLeft.setVelocity(fLvelocity);
        frontRight.setVelocity(fRvelocity);
        backLeft.setVelocity(bLvelocity);
        backRight.setVelocity(bRvelocity);
    }

    public void allTargetPosition(int frontLeftPos, int frontRightPos,
                                  int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
    }

    public void strafeNW(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, 0, inches, inches, 0, active, tOut);
    }

    public void fwdDrive(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, inches, inches, inches, inches, active, tOut);
    }

    public void strafeNE(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, inches, 0, 0, inches, active, tOut);
    }

    public void strafeLeft(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, -inches, inches, inches, -inches, active, tOut);
    }

    public void strafeRight(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, inches, -inches, -inches, inches, active, tOut);
    }

    public void turnCC(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, -inches, inches, -inches, inches, active, tOut);
    }

    public void turnCW(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, inches, -inches, inches, -inches, active, tOut);
        //7.06858347055 inches
    }

    public void turnAmount(int target, boolean active, Telemetry telemetry) {
        allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            if (active) {
                if (target < 0) {
                    while (Yaw > target) {
                        setMotorVelocity(-850, 850, -850, 850);
                        robotOrientation = imu.getRobotYawPitchRollAngles();
                        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.update();
                    }
                    telemetry.addData("Yaw: ", Yaw);
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    allMotorVelocity(0);
                    telemetry.addData("Yaw: ", Yaw);
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    if (Yaw < target) {
                        robotOrientation = imu.getRobotYawPitchRollAngles();
                        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                        double error = Yaw - target;
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.addData("Error: ", error);
                        telemetry.update();
                        while (Math.abs(error) > 0.2) {
                            setMotorVelocity(850, -850, 850, -850);
                            robotOrientation = imu.getRobotYawPitchRollAngles();
                            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                            error = Yaw - target;
                            telemetry.addData("Yaw: ", Yaw);
                            telemetry.addData("Target: ", target);
                            telemetry.addData("Error: ", error);
                            telemetry.update();
                        }
                        allMotorVelocity(0);
                    }
                }
                if (target > 0){
                    while (Yaw < target) {
                        setMotorVelocity(-850, 850, -850, 850);
                        robotOrientation = imu.getRobotYawPitchRollAngles();
                        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.update();
                    }
                    telemetry.addData("Yaw: ", Yaw);
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    allMotorVelocity(0);
                    telemetry.addData("Yaw: ", Yaw);
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    if (Yaw > target) {
                        robotOrientation = imu.getRobotYawPitchRollAngles();
                        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                        double error = Yaw - target;
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.addData("Error: ", error);
                        telemetry.update();
                        while (Math.abs(error) > 0.2) {
                            setMotorVelocity(850, -850, 850, -850);
                            robotOrientation = imu.getRobotYawPitchRollAngles();
                            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                            error = Yaw - target;
                            telemetry.addData("Yaw: ", Yaw);
                            telemetry.addData("Target: ", target);
                            telemetry.addData("Error: ", error);
                            telemetry.update();
                        }
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.addData("Error: ", error);
                        telemetry.update();
                        allMotorVelocity(0);
                        telemetry.addData("Yaw: ", Yaw);
                        telemetry.addData("Target: ", target);
                        telemetry.addData("Error: ", error);
                        telemetry.update();
                    }
                }
            }
        }

    public void strafeSW(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, -inches, 0, 0, -inches, active, tOut);
    }

    public void revDrive(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, -inches, -inches, -inches, -inches, active, tOut);
    }

    public void strafeSE(double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        Drive(vel, 0, -inches, -inches, 0, active, tOut);
    }

}
