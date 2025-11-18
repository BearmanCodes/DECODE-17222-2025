package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.io.IOException;
import java.util.Locale;

@Config
@TeleOp
public class IMU extends LinearOpMode {

    Limelight3A limelight;

    DrivetrainCore dtCore = new DrivetrainCore();

    GoBildaPinpointDriver odo;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            dtCore.init(hardwareMap);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        odo_init();
        waitForStart();
        while (!isStopRequested()){
            dtCore.run(gamepad1);
            odo.update();

            Pose2D pos = odo.getPosition();
            limelight.updateRobotOrientation(pos.getHeading(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose_mt2 = llResult.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    double yaw = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ", " + yaw + " )");
                    dashTele.addData("MT2 Location:", "(" + x + ", " + y + ", " + yaw + " )");
                }
                telemetry.addData("Tx: ", llResult.getTx());
                telemetry.addData("Ta: ", llResult.getTa());
                telemetry.addData("Ty: ", llResult.getTy());
                telemetry.addData("TxNC: ", llResult.getTxNC());
                telemetry.addData("TyNC: ", llResult.getTyNC());
                dashTele.addData("Tx: ", llResult.getTx());
                dashTele.addData("Ta: ", llResult.getTa());
                dashTele.addData("Ty: ", llResult.getTy());
                dashTele.addData("TxNC: ", llResult.getTxNC());
                dashTele.addData("TyNC: ", llResult.getTyNC());
                //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position X: ", odo.getEncoderX());
                telemetry.addData("Position Y: ", odo.getEncoderY());
                dashTele.addData("Position X: ", odo.getEncoderX());
                dashTele.addData("Position Y: ", odo.getEncoderY());
                //dashTele.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
                String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);
                telemetry.update();
                dashTele.addData("Velocity", velocity);
                dashTele.update();
            }

        }
    }

    public void odo_init(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, ":3");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(7.025, 0.95, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1 //CHANGE THIS

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED); //CHANGE THIS

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
        dashTele.addData("Status", "Initialized");
        dashTele.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        dashTele.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        dashTele.addData("Heading Scalar", odo.getYawScalar());
        dashTele.update();

        limelight.start();
    }
}
