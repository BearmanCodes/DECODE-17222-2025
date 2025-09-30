package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTestCore {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    ArmAutoCore armCore = new ArmAutoCore();
    Action action = new Action();

    YawPitchRollAngles robotOrientation;
    public IMU imu;
    public IMU.Parameters imuparams;
    private ElapsedTime runtime = new ElapsedTime();

    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);

    public enum DriveDirection {
        FWD,
        REV,
        NW,
        NE,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_CC,
        TURN_CW,
        SW,
        SE
    }

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

    public void ArmColqDrive(double velocity, double armVelo,
                      double frontLeftInches, double frontRightInches,
                      double backLeftInches, double backRightInches, int armTicks, boolean active,
                      long timeout) throws InterruptedException {
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;
        int frontLeftError;
        int frontRightError;
        int backLeftError;
        int backRightError;
        int armError;
        int armPos;
        int frontLeftPos;
        int frontRightPos;
        int backLeftPos;
        int backRightPos;
        int errTolerance = 5;

        // Ensure that the opmode is still active
        if (active) {
            runtime.reset();
            frontLeftPos = frontLeft.getCurrentPosition();
            frontRightPos = frontRight.getCurrentPosition();
            backLeftPos = backLeft.getCurrentPosition();
            backRightPos = backRight.getCurrentPosition();
            armPos = armCore.pvtArm.getCurrentPosition();
            frontLeftTarget = frontLeftPos + (int) (frontLeftInches * TicksPerIn);
            frontRightTarget = frontRightPos + (int) (frontRightInches * TicksPerIn);
            backLeftTarget = backLeftPos + (int) (backLeftInches * TicksPerIn);
            backRightTarget = backRightPos + (int) (backRightInches * TicksPerIn);

            allTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            armCore.pvtArm.setTargetPosition(armTicks);

            allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            armCore.pvtArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            allMotorVelocity(Math.abs(velocity));
            armCore.pvtArm.setVelocity(Math.abs(armVelo));

            frontLeftError = Math.abs(frontLeftPos - frontLeftTarget);
            frontRightError = Math.abs(frontRightPos - frontRightTarget);
            backLeftError = Math.abs(backLeftPos - backLeftTarget);
            backRightError = Math.abs(backRightPos - backRightTarget);
            armError = Math.abs(armPos - armTicks);
            boolean motorComplete = frontLeftError <= errTolerance && frontRightError <= errTolerance
                    &&  backLeftError <= errTolerance && backRightError <= errTolerance;
            int cumulativeError = frontLeftError + frontRightError + backLeftError + backRightError;

            if (motorComplete) {
                allMotorVelocity(0);
                allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (armError <= 1){
                armCore.pvtArm.setVelocity(0);
                armCore.pvtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

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

    public void drive(DriveDirection direction, double vel, double inches, boolean active, long tOut) throws InterruptedException {
        inches = Math.abs(inches);
        vel = Math.abs(vel);
        switch (direction){
            case NW:
                Drive(vel, 0, inches, inches, 0, active, tOut);
                break;
            case FWD:
                Drive(vel, inches, inches, inches, inches, active, tOut);
                break;
            case REV:
                Drive(vel, -inches, -inches, -inches, -inches, active, tOut);
                break;
            case NE:
                Drive(vel, inches, 0, 0, inches, active, tOut);
                break;
            case STRAFE_LEFT:
                Drive(vel, -inches, inches, inches, -inches, active, tOut);
                break;
            case STRAFE_RIGHT:
                Drive(vel, inches, -inches, -inches, inches, active, tOut);
                break;
            case TURN_CC:
                Drive(vel, -inches, inches, -inches, inches, active, tOut);
                break;
            case TURN_CW:
                Drive(vel, inches, -inches, inches, -inches, active, tOut);
                break;
            case SW:
                Drive(vel, -inches, 0, 0, -inches, active, tOut);
                break;
            case SE:
                Drive(vel, 0, -inches, -inches, 0, active, tOut);
                break;
        }
    }

    public void allTargetPosition(int frontLeftPos, int frontRightPos,
                                  int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
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

}
