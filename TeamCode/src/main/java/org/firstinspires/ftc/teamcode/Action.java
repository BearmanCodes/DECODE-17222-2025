package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Action {
    private ElapsedTime timer = new ElapsedTime();
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
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();
    public static double driveTol = 1;
    public static double armTol = 1;
    public static double wristTol = 10;

    public interface Executable {
        void run(ElapsedTime time);
        double getPeriod();
        boolean getRunCondition();
        void setupParams();
    }

    public static class Drive implements Executable {
        private DriveAutoCore driveCore = new DriveAutoCore();
        static final double TicksPerRev = 560;
        static final double WheelInches = (75 / 25.4);
        static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);
        private DriveDirection dir;
        private double inches;
        private double velocity;
        public double period = 0;
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;
        public boolean canRun = true;

        public Drive setDir(DriveDirection dir){
            this.dir = dir;
            return this;
        }

        public Drive setVelocity(double vel){
            this.velocity = vel;
            return this;
        }

        public Drive setInches(double inches){
            this.inches = inches;
            return this;
        }

        public Drive setPeriod(double period){
            this.period = period;
            return this;
        }

        public Drive (DriveAutoCore core){
            this.driveCore = core;
        }

        @Override
        public double getPeriod(){
            return this.period;
        }

        @Override
        public boolean getRunCondition(){
            return this.canRun;
        }

        @Override
        public void setupParams(){
            switch (dir){
                case NW:
                    frontLeftTarget = 0;
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backRightTarget = 0;
                    break;
                case FWD:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    break;
                case REV:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    break;
                case NE:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    frontRightTarget = 0;
                    backLeftTarget = 0;
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    break;
                case STRAFE_LEFT:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    break;
                case STRAFE_RIGHT:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    break;
                case TURN_CC:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (inches * TicksPerIn);
                    break;
                case TURN_CW:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (inches * TicksPerIn);
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    break;
                case SW:
                    frontLeftTarget = driveCore.frontLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    frontRightTarget = 0;
                    backLeftTarget = 0;
                    backRightTarget = driveCore.backRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    break;
                case SE:
                    frontLeftTarget = 0;
                    frontRightTarget = driveCore.frontRight.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backLeftTarget = driveCore.backLeft.getCurrentPosition() + (int) (-inches * TicksPerIn);
                    backRightTarget = 0;
                    break;
            }
            this.canRun = true;
            dashTele.addData("FL: ", frontLeftTarget);
            dashTele.addData("FR: ", frontRightTarget);
            dashTele.addData("BL: ", backLeftTarget);
            dashTele.addData("BR: ", backRightTarget);
            dashTele.update();
            driveCore.allTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            driveCore.allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        @Override
        public void run(ElapsedTime time){
            dashTele.addData("Drive setup: ", true);
            dashTele.addData("Seconds: ", time.seconds());
            dashTele.addData("Period: ", period);
            dashTele.update();
            int flErr = Math.abs(frontLeftTarget - driveCore.frontLeft.getCurrentPosition());
            int frErr = Math.abs(frontRightTarget - driveCore.frontRight.getCurrentPosition());
            int blErr = Math.abs(backLeftTarget - driveCore.backLeft.getCurrentPosition());
            int brErr = Math.abs(backRightTarget - driveCore.backRight.getCurrentPosition());
            if (time.milliseconds() >= period){
                if (flErr > driveTol){
                    driveCore.frontLeft.setVelocity(velocity);
                    dashTele.addData("flErr: ", flErr);
                    dashTele.addData("Should be driving right now", true);
                    dashTele.addData("Drive setup: ", false);
                    dashTele.update();
                } else if (flErr <= driveTol){
                    driveCore.frontLeft.setVelocity(0);
                }
                if (frErr > driveTol){
                    driveCore.frontRight.setVelocity(velocity);
                    dashTele.addData("frErr: ", frErr);
                    dashTele.addData("Should be driving right now", true);
                    dashTele.addData("Drive setup: ", false);
                    dashTele.update();
                } else if (frErr <= driveTol){
                    driveCore.frontRight.setVelocity(0);
                }
                if (blErr > driveTol){
                    driveCore.backLeft.setVelocity(velocity);
                    dashTele.addData("blErr: ", blErr);
                    dashTele.addData("Should be driving right now", true);
                    dashTele.addData("Drive setup: ", false);
                    dashTele.update();
                } else if (blErr <= driveTol){
                    driveCore.backLeft.setVelocity(0);
                }
                if (brErr > driveTol){
                    driveCore.backRight.setVelocity(velocity);
                    dashTele.addData("brErr: ", brErr);
                    dashTele.addData("Should be driving right now", true);
                    dashTele.addData("Drive setup: ", false);
                    dashTele.update();
                } else if (brErr <= driveTol){
                    driveCore.backRight.setVelocity(0);
                }
                else {
                    dashTele.addData("Should be driving right now", false);
                    dashTele.addData("Drive setup: ", false);
                    dashTele.update();
                    driveCore.allMotorVelocity(0);
                    driveCore.allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.canRun = false;
                }
            }
        }
    }
    public static class Arm implements Executable{
        public int ticks;
        public double velocity;
        public double period = 0;
        public ArmAutoCore armCore;
        boolean canRun = true;

        public Arm setVelocity(double vel){
            this.velocity = vel;
            return this;
        }

        public Arm setTicks(int ticks){
            this.ticks = ticks;
            return this;
        }

        public Arm setPeriod(double period){
            this.period = period;
            return this;
        }

        public Arm (ArmAutoCore core){
            this.armCore = core;
        }

        @Override
        public double getPeriod() {
            return this.period;
        }

        public boolean getRunCondition(){
            return this.canRun;
        }

        @Override
        public void setupParams(){
            armCore.pvtArm.setTargetPosition(ticks);
            armCore.pvtArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.canRun = true;
        }

        @Override
        public void run(ElapsedTime time){
            int err = Math.abs(ticks - armCore.pvtArm.getCurrentPosition());

            if (time.milliseconds() >= period){
                if (err > armTol){
                    armCore.pvtArm.setVelocity(velocity);
                } else {
                    armCore.pvtArm.setVelocity(0);
                    armCore.pvtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.canRun = false;
                }
            }
        }
    }

    public void run(boolean active, Executable... Actions){
        if (active){
            int count = Actions.length - 1;
            timer.reset();
            for (Executable action: Actions){
                action.setupParams();
            }
            while (count > 0){
                for (Executable action: Actions){
                    if (action.getRunCondition()){
                        action.run(timer);
                        count+= 1;
                    } else {
                        count -= 1;
                    }
                }
            }
        }

    }
}