package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.hardware.Servo;
//0.04 rest up
//0.25 start home
//0.06 close
//0.47/8 rest down
//
@TeleOp(name = "ServoTest")
public class servoTest extends LinearOpMode {
    Servo servo1iamgoingtonewjersey, servo2iamgoingtonewjersey, servo3iamgoingtonewjersey, servo4iamgoingtonewjersey;
          //1        2        4       3
    Gamepad currentGamepad =  new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    ServoImplEx la;

    private static final DecimalFormat dformat = new DecimalFormat("0.00");
    double servo1iamgoingtonewjerseyPos, servo2iamgoingtonewjerseyPos, servo3iamgoingtonewjerseyPos, servo4iamgoingtonewjerseyPos, laPos;
    @Override

    public void runOpMode() throws InterruptedException {
        servo1iamgoingtonewjersey = hardwareMap.get(Servo.class, "servecunt1iamgoingtokillmyself".toLowerCase());
        servo2iamgoingtonewjersey = hardwareMap.get(Servo.class, "servecunt2iamgoingtokillmyself".toLowerCase());
        servo3iamgoingtonewjersey = hardwareMap.get(Servo.class, "servecunt3iamgoingtokillmyself".toLowerCase());
        servo4iamgoingtonewjersey = hardwareMap.get(Servo.class, "servecunt4iamgoingtokillmyself".toLowerCase());
        la = hardwareMap.get(ServoImplEx.class, "la");

        la.setPwmRange(new PwmControl.PwmRange(1000, 2000));
                                                //fully retract 0, fully extend 1
        la.setPwmEnable();

        waitForStart();
        while (opModeIsActive()){
            try {
                edgeDetector();
            } catch (RobotCoreException e) {
                throw new RuntimeException(e);
            }
            if (currentGamepad.y && !previousGamepad.y){
                laPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.x && !previousGamepad.x){
                laPos -= 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up){
                servo2iamgoingtonewjerseyPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down){
                servo2iamgoingtonewjerseyPos -= 0.01;
                telemetryUpdate();
            }

            if (currentGamepad.a && !previousGamepad.a){
                servo3iamgoingtonewjerseyPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.b && !previousGamepad.b) {
                servo3iamgoingtonewjerseyPos -= 0.01;
                telemetryUpdate();
            }

            if (currentGamepad.right_bumper && !previousGamepad.right_bumper){
                servo4iamgoingtonewjerseyPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                servo4iamgoingtonewjerseyPos -= 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.start && !previousGamepad.start){
                servo1iamgoingtonewjersey.setPosition(servo1iamgoingtonewjerseyPos); //0.04 open, 0 closed
                servo2iamgoingtonewjersey.setPosition(servo2iamgoingtonewjerseyPos); //0.04 open, 0 closed
                servo3iamgoingtonewjersey.setPosition(servo3iamgoingtonewjerseyPos); //0.04 open, 0 closed
                servo4iamgoingtonewjersey.setPosition(servo4iamgoingtonewjerseyPos); //0.04 open, 0 closed
                la.setPosition(laPos);
                telemetryUpdate();
            }
        }
    }

    public void telemetryUpdate(){
        telemetry.addData("First Servo Position", dformat.format(laPos));
        telemetry.addData("Second Servo Position", dformat.format(servo2iamgoingtonewjerseyPos));
        telemetry.addData("Third Servo Position", dformat.format(servo3iamgoingtonewjerseyPos));
        telemetry.addData("Fourth Second Servo Position", dformat.format(servo4iamgoingtonewjerseyPos));

        telemetry.addData("Current First Servo Position", dformat.format(la.getPosition()));
        telemetry.addData("Current Second Servo Position", dformat.format(servo2iamgoingtonewjersey.getPosition()));
        telemetry.addData("Current Third Servo Position", dformat.format(servo3iamgoingtonewjersey.getPosition()));
        telemetry.addData("Current Fourth Second Servo Position", dformat.format(servo4iamgoingtonewjersey.getPosition()));

        telemetry.update();
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
