package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.Servo;
//0.04 rest up
//0.25 start home
//0.06 close
//0.47/8 rest down
@TeleOp(name = "ServoTest")
public class servoTest extends LinearOpMode {
    Servo servecunt1iamgoingtokillmyself, servecunt2iamgoingtokillmyself, servecunt3iamgoingtokillmyself, servecunt4iamgoingtokillmyself;
          //1        2        4       3
    Gamepad currentGamepad =  new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    private static final DecimalFormat dformat = new DecimalFormat("0.00");
    double servecunt1iamgoingtokillmyselfPos, servecunt2iamgoingtokillmyselfPos, servecunt3iamgoingtokillmyselfPos, servecunt4iamgoingtokillmyselfPos;
    @Override

    public void runOpMode() throws InterruptedException {
        servecunt1iamgoingtokillmyself = hardwareMap.get(Servo.class, "servecunt1iamgoingtokillmyself".toLowerCase());
        servecunt2iamgoingtokillmyself = hardwareMap.get(Servo.class, "servecunt2iamgoingtokillmyself".toLowerCase());
        servecunt3iamgoingtokillmyself = hardwareMap.get(Servo.class, "servecunt3iamgoingtokillmyself".toLowerCase());
        servecunt4iamgoingtokillmyself = hardwareMap.get(Servo.class, "servecunt4iamgoingtokillmyself".toLowerCase());

        waitForStart();
        while (opModeIsActive()){
            try {
                edgeDetector();
            } catch (RobotCoreException e) {
                throw new RuntimeException(e);
            }
            if (currentGamepad.y && !previousGamepad.y){
                servecunt1iamgoingtokillmyselfPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.x && !previousGamepad.x){
                servecunt1iamgoingtokillmyselfPos -= 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up){
                servecunt2iamgoingtokillmyselfPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down){
                servecunt2iamgoingtokillmyselfPos -= 0.01;
                telemetryUpdate();
            }

            if (currentGamepad.a && !previousGamepad.a){
                servecunt3iamgoingtokillmyselfPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.b && !previousGamepad.b) {
                servecunt3iamgoingtokillmyselfPos -= 0.01;
                telemetryUpdate();
            }

            if (currentGamepad.right_bumper && !previousGamepad.right_bumper){
                servecunt4iamgoingtokillmyselfPos += 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                servecunt4iamgoingtokillmyselfPos -= 0.01;
                telemetryUpdate();
            }
            if (currentGamepad.start && !previousGamepad.start){
                servecunt1iamgoingtokillmyself.setPosition(servecunt1iamgoingtokillmyselfPos); //0.04 open, 0 closed
                servecunt2iamgoingtokillmyself.setPosition(servecunt2iamgoingtokillmyselfPos); //0.04 open, 0 closed
                servecunt3iamgoingtokillmyself.setPosition(servecunt3iamgoingtokillmyselfPos); //0.04 open, 0 closed
                servecunt4iamgoingtokillmyself.setPosition(servecunt4iamgoingtokillmyselfPos); //0.04 open, 0 closed

                telemetryUpdate();
            }
        }
    }

    public void telemetryUpdate(){
        telemetry.addData("First Servo Position", dformat.format(servecunt1iamgoingtokillmyselfPos));
        telemetry.addData("Second Servo Position", dformat.format(servecunt2iamgoingtokillmyselfPos));
        telemetry.addData("Third Servo Position", dformat.format(servecunt3iamgoingtokillmyselfPos));
        telemetry.addData("Fourth Second Servo Position", dformat.format(servecunt4iamgoingtokillmyselfPos));

        telemetry.addData("Current First Servo Position", dformat.format(servecunt1iamgoingtokillmyself.getPosition()));
        telemetry.addData("Current Second Servo Position", dformat.format(servecunt2iamgoingtokillmyself.getPosition()));
        telemetry.addData("Current Third Servo Position", dformat.format(servecunt3iamgoingtokillmyself.getPosition()));
        telemetry.addData("Current Fourth Second Servo Position", dformat.format(servecunt4iamgoingtokillmyself.getPosition()));

        telemetry.update();
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
