package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.text.DecimalFormat;

//0.04 rest up
//0.25 start home
//0.06 close
//0.47/8 rest down
@TeleOp(name = "ServoTest")
public class clawTest extends LinearOpMode {
    public CRServo firstservo, secondservo;
          //1        2        4       3
    Gamepad currentGamepad =  new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    private static final DecimalFormat dformat = new DecimalFormat("0.00");
    double servo1Pos, servo2Pos, servo3Pos, servo4Pos;
    @Override

    public void runOpMode() throws InterruptedException {
        firstservo = hardwareMap.get(CRServo.class, "ServoLeft");
        secondservo = hardwareMap.get(CRServo.class, "ServoRight");
        firstservo.setDirection(CRServo.Direction.FORWARD);
        secondservo.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            firstservo.setPower(1);
            secondservo.setPower(1);
        }
    }
    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
