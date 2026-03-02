package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.text.DecimalFormat;
//0.04 rest up
//0.25 start home
//0.06 close
//0.47/8 rest down
//

@Config
@TeleOp(name = "Swyft")
public class swyftTesting extends LinearOpMode {
    ServoImplEx swyft;

    Gamepad currentGamepad =  new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    public static Servo.Direction dir;
    public static boolean isFwd = false;

    double laPos;

    public static double firstTogPos;

    public static double secondTogPos;

    boolean toggle = false;

    private static final DecimalFormat dformat = new DecimalFormat("0.00");

    @Override

    public void runOpMode() throws InterruptedException {
        swyft = hardwareMap.get(ServoImplEx.class, "swyft");

        //la = hardwareMap.get(ServoImplEx.class, "la");

        //swyft.setPwmRange(new PwmControl.PwmRange(500, 2500));
                 //fully retract 0, fully extend 1
        //swyft.setPwmEnable();

        dir = isFwd ? Servo.Direction.FORWARD : Servo.Direction.REVERSE;

        swyft.setDirection(dir);

        waitForStart();
        while (opModeIsActive()){
            try {
                edgeDetector();
            } catch (RobotCoreException e) {
                throw new RuntimeException(e);
            }
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper){
                laPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper){
                laPos -= 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.start && !previousGamepad.start){
                swyft.setPosition(laPos);
                telemetryUpdate();
            }
            if (currentGamepad.left_stick_button && !previousGamepad.left_stick_button){
                firstTogPos = laPos;
                telemetryUpdate();
            }
            if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button){
                secondTogPos = laPos;
                telemetryUpdate();
            }
            if (currentGamepad.cross && !previousGamepad.cross){
                toggle = !toggle;
                if (toggle){
                    swyft.setPosition(firstTogPos);
                } else {
                    swyft.setPosition(secondTogPos);
                }
                telemetryUpdate();
            }
        }
    }

    public void telemetryUpdate(){
        telemetry.addData("First Servo Position", dformat.format(laPos));
        telemetry.addData("Toggle True Position", dformat.format(firstTogPos));
        telemetry.addData("Toggle False Position", dformat.format(secondTogPos));
        telemetry.addData("Next toggle: ", !toggle);

        telemetry.update();
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
