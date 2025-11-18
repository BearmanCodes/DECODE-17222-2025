package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Drive extends LinearOpMode {
    private DrivetrainCore dtCore = new DrivetrainCore();

    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    private static final DecimalFormat dformat = new DecimalFormat("0.00");

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad(); //Set up gamepad variables allowing for rising edge detector
    private CRServo lServo, rServo, tempServo;

    public ServoImplEx la;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double lPower = 1100;
    public static double rPower = 1100;

    public static double lServoPower = 1.0;

    public static double rServoPower = 1.0;

    public static double tempServoPower = 1.0;

    public static double inPower = 750;

    public static boolean inFWD = false;

    public static boolean tempFWD = false;

    public static double laIter = 0.05;

    public static double laPos = 0;

    public static boolean crStat = false;

    public static boolean flyStat = false;



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
        tempServo = hardwareMap.get(CRServo.class, "temp");
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction tempDir = tempFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        tempServo.setDirection(tempDir);
        intake.setDirection(inDir);
        la = hardwareMap.get(ServoImplEx.class, "la");

        la.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        //fully retract 0, fully extend 1
        la.setPwmEnable();
        la.setPosition(0);

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

        InitFile();
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                dtCore.run(gamepad1);
                edgeDetector(gamepad1, gamepad2);
                //double mPower = gamepad1.left_stick_y;
                //lPower = gamepad1.left_trigger;
                //rPower = gamepad1.right_trigger;

                inPower = gamepad1.left_trigger - gamepad1.right_trigger;
                intake.setPower(inPower);

                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    double currPos = Math.round(la.getPosition() * 100.00) / 100.00;

                    la.setPosition(currPos + laIter);
                    dashTele.addData("LA Pos: ", la.getPosition());
                    dashTele.update();
                }

                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    double currPos = Math.round(la.getPosition() * 100.00) / 100.00;
                    la.setPosition(currPos - laIter);
                    dashTele.addData("LA Pos: ", la.getPosition());
                    dashTele.update();
                }

                if (currentGamepad.x && !previousGamepad.x) {
                    crStat = !crStat;
                    if (crStat) {
                        lServo.setPower(lServoPower);
                        rServo.setPower(rServoPower);
                        tempServo.setPower(tempServoPower);
                    } else {
                        lServo.setPower(0);
                        rServo.setPower(0);
                        tempServo.setPower(0);
                    }
                }

                if (currentGamepad.a && !previousGamepad.a) {
                    flyStat = !flyStat;
                    if (flyStat) {
                        fly.setVelocity(lPower);
                        fry.setVelocity(rPower);
                    } else {
                        fly.setPower(0);
                        fry.setPower(0);
                    }
                }

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
                dashTele.update();
                //AprilTagDetect(telemetry);

            }
        }
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
            bw.write("X,Y,Z,Pitch,Roll,Yaw,Range,Bearing,Elevation,L vel,R vel,LA Pos");
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

    public void AppendData(String output){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("yourmom.csv"))) {
            bw.newLine();
            bw.write(output);
            telemetry.addLine("Successfully wrote to the file.");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("Error writing file.");
            telemetry.update();
        }
    }

}
