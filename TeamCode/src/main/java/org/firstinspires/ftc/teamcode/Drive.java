package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Drive extends LinearOpMode {
    public static double laTYQuotienter = 30.0;

    Limelight3A limelight;

    DrivetrainCore dtCore = new DrivetrainCore();

    GoBildaPinpointDriver odo;

    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    private static final DecimalFormat dformat = new DecimalFormat("0.00");

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad(); //Set up gamepad variables allowing for rising edge detector
    private CRServo lServo, rServo, luigiCont;

    private Servo luigiServo;

    public ServoImplEx laR, laL;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double lPower = 1100;
    public static double rPower = 1100;

    public static double intakeReducer = 0.15;

    public static double lServoPower = 1.0;

    public static double rServoPower = 1.0;

    //public static double tempServoPower = 1.0;

    public static double inPower = 750;

    public static double luigiBlock = 0.515;

    public static double luigiFlow = 0.19;


    public static double luigiContPower = 1;

    public static double laInitPos = 0.2;

    public static boolean inFWD = false;

    //public static boolean tempFWD = false;

    public static double laIter = 0.05;

    public static double lalPos = 0;

    public static double larPos = 0;


    public static boolean crStat = false;

    public static double surge_measure = 150;

    public static boolean flyStat = false;

    public static boolean luigiStat = true;

    double limelightMountAngleDegrees = 0;
    double limelightLensHeightInches = 15.75;
    double goalHeightInches = 32;


    private DcMotorEx fly, fry, intake;

    private AprilTagProcessor aTag;

    public List<Integer> valid_ids = Arrays.asList(21, 22, 23, 20, 24);

    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            dtCore.init(hardwareMap);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        lServo = hardwareMap.get(CRServo.class, "crl");
        rServo = hardwareMap.get(CRServo.class, "crr");
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fry = hardwareMap.get(DcMotorEx.class, "fry");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        luigiServo = hardwareMap.get(Servo.class, "WEARE");
        luigiCont = hardwareMap.get(CRServo.class, "bubba");
        luigiServo.setPosition(luigiBlock); //0.19 is back enough that balls fall, //0.52 is straight block
        luigiCont.setPower(0);

        //tempServo = hardwareMap.get(CRServo.class, "temp");
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        luigiCont.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
       // DcMotorSimple.Direction tempDir = tempFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        //tempServo.setDirection(tempDir);
        intake.setDirection(inDir);
        laR = hardwareMap.get(ServoImplEx.class, "lar");

        laR.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        laR.setPwmEnable();
        laR.setPosition(laInitPos);

        laL = hardwareMap.get(ServoImplEx.class, "lal");

        laL.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        laL.setPwmEnable();
        laL.setPosition(laInitPos);



        /*
        aTag = new AprilTagProcessor.Builder().setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                .build();
        aTag.setDecimation(1);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.enableLiveView(true);

        builder.addProcessor(aTag);

        visionPortal = builder.build();
         */

        init_odo();
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                dtCore.run(gamepad1);
                edgeDetector(gamepad1, gamepad2);
                odo.update();

                Pose2D pos = odo.getPosition();
                limelight.updateRobotOrientation(pos.getHeading(AngleUnit.DEGREES));

                LLResult llResult = limelight.getLatestResult();



                if (llResult != null && llResult.isValid()) {
                /*
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    double xPix = fiducial.getTargetXPixels(); // Where it is (left-right)
                    double xDeg = fiducial.getTargetXDegrees();
                    double yPix = fiducial.getTargetYPixels(); // Where it is (up-down)
                    double yDeg = fiducial.getTargetYDegrees();
                    telemetry.addData("xPix: ", xPix);
                    telemetry.addData("xDeg: ", xDeg);
                    telemetry.addData("yPix: ", yPix);
                    telemetry.addData("yDeg: ", yDeg);
                    dashTele.addData("xPix: ", xPix);
                    dashTele.addData("xDeg: ", xDeg);
                    dashTele.addData("yPix: ", yPix);
                    dashTele.addData("yDeg: ", yDeg);
                }
                */

                    double targetOffsetAngle_Vertical = Math.round(llResult.getTy() * 100.00) / 100.00;
                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
                    double distanceFromLimelightToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                    Pose3D botpose_mt2 = llResult.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        telemetry.addLine("READY TO EXPORT DATA");
                        dashTele.addLine("READY TO EXPORT DATA");
                            double Tx = llResult.getTx();
                            double Ty = llResult.getTy();
                            double Ta = llResult.getTa();
                            double Distance = distanceFromLimelightToGoal;
                            double M2_X = botpose_mt2.getPosition().x;
                            double M2_Y = botpose_mt2.getPosition().y;
                            double M2_Head = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                            double Odo_X = pos.getX(DistanceUnit.MM);
                            double Odo_Y = pos.getY(DistanceUnit.MM);
                            double Odo_Head = pos.getHeading(AngleUnit.DEGREES);
                            double L_Vel = lPower;
                            double R_Vel = rPower;
                            double L_Vel_Measure = fly.getVelocity();
                            double R_Vel_Measure = fry.getVelocity();
                    }

                    /*
                    if (llResult.getTy() < 2){
                        double laPos = 0;
                        la.setPosition(laPos);
                    } else {
                        int intTY = (int) llResult.getTy();
                        double laDouble = (double) intTY / laTYQuotienter;
                        double laPos = Math.round(laDouble * 100.00) / 100.00;
                        la.setPosition(laPos);
                    }
                     */


                    telemetry.addData("Tx: ", llResult.getTx());
                    telemetry.addData("Ta: ", llResult.getTa());
                    telemetry.addData("Ty: ", llResult.getTy());
                    telemetry.addData("TxNC: ", llResult.getTxNC());
                    telemetry.addData("TyNC: ", llResult.getTyNC());
                    telemetry.addData("Distance: ", distanceFromLimelightToGoal);
                    dashTele.addData("Tx: ", llResult.getTx());
                    dashTele.addData("Ta: ", llResult.getTa());
                    dashTele.addData("Ty: ", llResult.getTy());
                    dashTele.addData("TxNC: ", llResult.getTxNC());
                    dashTele.addData("TyNC: ", llResult.getTyNC());
                    dashTele.addData("Distance: ", distanceFromLimelightToGoal);
                    String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                    dashTele.addData("Position: ", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
                    String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                    telemetry.addData("Velocity", velocity);
                    telemetry.update();
                    dashTele.addData("Velocity", velocity);
                    dashTele.update();
                }

                //double mPower = gamepad1.left_stick_y;
                //lPower = gamepad1.left_trigger;
                //rPower = gamepad1.right_trigger;
                //.5
                inPower = gamepad1.left_trigger - gamepad1.right_trigger;
                intake.setPower(inPower * intakeReducer);

                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    double currPosL = Math.round(laL.getPosition() * 100.00) / 100.00;
                    double currPosR = Math.round(laR.getPosition() * 100.00) / 100.00;

                    laL.setPosition(currPosL + laIter);
                    laR.setPosition(currPosR + laIter);
                    dashTele.addData("LAl Pos: ", laL.getPosition());
                    dashTele.addData("LAr Pos: ", laR.getPosition());
                    dashTele.update();
                }

                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    double currPosL = Math.round(laL.getPosition() * 100.00) / 100.00;
                    double currPosR = Math.round(laR.getPosition() * 100.00) / 100.00;

                    laL.setPosition(currPosL - laIter);
                    laR.setPosition(currPosR - laIter);
                    dashTele.addData("LAl Pos: ", laL.getPosition());
                    dashTele.addData("LAr Pos: ", laR.getPosition());
                    dashTele.update();
                }

                if (currentGamepad.x && !previousGamepad.x) {
                    crStat = !crStat;
                    if (crStat) {
                        lServo.setPower(lServoPower);
                        rServo.setPower(rServoPower);
                        //tempServo.setPower(tempServoPower);
                    } else {
                        lServo.setPower(0);
                        rServo.setPower(0);
                        //tempServo.setPower(0);
                    }
                }


                if (currentGamepad.b && !previousGamepad.b) {
                    luigiStat = !luigiStat;
                    if (luigiStat) {
                        luigiServo.setPosition(luigiBlock);
                    } else {
                        luigiServo.setPosition(luigiFlow);
                    }
                }

                if (currentGamepad.a && !previousGamepad.a && !currentGamepad.start) {
                    flyStat = !flyStat;
                    if (flyStat) {
                        fly.setVelocity(lPower);
                        fry.setVelocity(rPower);
                    } else {
                        fly.setPower(0);
                        fry.setPower(0);
                    }
                }


                power_surge(lPower, rPower, fly.getVelocity(), fry.getVelocity(), surge_measure);

                telemetry.addData("L Power: ", lPower);
                telemetry.addData("L RPM: ", 6000 * lPower);
                telemetry.addData("L Output Velocity: ", fly.getVelocity());

                telemetry.addData("R Power: ", rPower);
                telemetry.addData("R RPM: ", 6000 * rPower);
                telemetry.addData("R Output Velocity: ", fry.getVelocity());
                telemetry.update();

                dashTele.addData("L Power: ", lPower);
                dashTele.addData("L RPM: ", 6000 * lPower);
                dashTele.addData("L Output Velocity: ", fly.getVelocity());
                dashTele.addData("time passed: ", timer.now(TimeUnit.MILLISECONDS));

                dashTele.addData("R Power: ", rPower);
                dashTele.addData("R RPM: ", 6000 * rPower);
                dashTele.addData("R Output Velocity: ", fry.getVelocity());

                dashTele.addData("Fl Pos: ", DrivetrainCore.frontleft.getCurrentPosition());
                dashTele.addData("Bl Pos: ", DrivetrainCore.backleft.getCurrentPosition());
                dashTele.addData("Fr Pos: ", DrivetrainCore.frontright.getCurrentPosition());
                dashTele.addData("Br Pos: ", DrivetrainCore.backright.getCurrentPosition());


                dashTele.update();
                //AprilTagDetect(telemetry);

            }
        }
    }

    public void power_surge(double lPower, double rPower, double lMeasured, double rMeasured, double surge_measure){
        if (lPower - lMeasured >= surge_measure && rPower - rMeasured >= surge_measure && lMeasured >= 200 && rMeasured >= 200 && lServo.getPower() > 0){
            crStat = !crStat;
            lServo.setPower(0);
            rServo.setPower(0);
        }
    }

    public void init_odo(){
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

    /*
    public void AprilTagDetect(Telemetry telemetry){
        List<AprilTagDetection> currentDetections = aTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && valid_ids.contains(detection.id)) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.update();
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                telemetry.update();
            }
            //                             x,y,z,pitch,roll,yaw,range,bearing,elevation
            String output = String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw, detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation, lPower, rPower, laPos);
            telemetry.addLine(output);
            if (gamepad1.a && !gamepad1.aWasPressed()){
                AppendData(output);
            }
            telemetry.update();
        }   // end for() loop
    }

     */

    public void InitFile(){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("yourmom.csv"))) {
            bw.write("Tx,Ty,Ta,Distance,M2 X,M2 Y,M2 Head,Odo X,Odo Y,Odo Head,L vel,R vel,L vel measure,R vel measure, LA Pos");
            System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
            System.out.println("Error writing file.");
        }
    }

    public void edgeDetector(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void GoodShot(double Tx, double Ty, double Ta, double Distance, double M2_x, double M2_Y, double M2_Head, double Odo_X, double Odo_Y, double Odo_Head, double L_vel, double R_vel, double L_real_vel, double R_real_vel, double LA_Pos){
        String data = Tx + "," + Ty + "," + Ta + "," + Distance + "," + M2_x + "," + M2_Y + "," + M2_Head + "," + Odo_X + "," + Odo_Y + "," + Odo_Head + "," + L_vel + "," + R_vel + "," + L_real_vel + "," + R_real_vel + "," + LA_Pos;
        AppendData(data);
    }

    public void AppendData(String output){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("yourmom.csv"))) {
            bw.newLine();
            bw.write(output);
            telemetry.addLine("Successfully wrote to the file.");
            telemetry.update();
            dashTele.addLine("Successfully wrote to the file.");
            dashTele.update();
        } catch (IOException e) {
            telemetry.addLine("Error writing file.");
            telemetry.update();
            dashTele.addLine("Error writing file.");
            dashTele.update();
        }
    }

}
