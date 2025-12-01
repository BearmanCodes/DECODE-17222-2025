package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.IOException;
import java.text.DecimalFormat;

public class TurnTest extends LinearOpMode {
    public DrivetrainCore dtCore = new DrivetrainCore();
    public OdometryCore odoCore = new OdometryCore();

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry dashTele = dashboard.getTelemetry();

    public static double TxToleration = 2.0;

    public LimelightCore limelightCore = new LimelightCore();

    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Init();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (!isStopRequested()){
            dtCore.run(gamepad1);
            OdometryCore.odo.update();
            Pose2D pos = OdometryCore.odo.getPosition();
            LimelightCore.limelight.updateRobotOrientation(pos.getHeading(AngleUnit.DEGREES));
            LLResult llresult = LimelightCore.get_result();
            edgeDetector(gamepad1, gamepad2);
        }
    }

    public void turn(LLResult llResult){
        llResult = LimelightCore.get_result();
        try{
            if (Math.abs(llResult.getTx()) > 2){

            }
        } catch (NullPointerException ignored){
            telemetry.addData("EXCEPTED: ", true);
            telemetry.update();
            dashTele.addData("EXCEPTED: ", true);
            dashTele.update();
        }
    }

    public void edgeDetector(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void Init() throws IOException {
        dtCore.init(hardwareMap);
        odoCore.init(hardwareMap);
        limelightCore.init(hardwareMap);
    }
}
