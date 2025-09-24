package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Drive extends LinearOpMode {
    //private DrivetrainCore dtCore = new DrivetrainCore();
    private CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        //dtCore.init(hardwareMap);
        servo = hardwareMap.get(CRServo.class, "cr");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
               // dtCore.run(gamepad1);
                if (gamepad1.x && !gamepad1.xWasPressed()){
                    servo.setPower(1);
                    telemetry.addLine("Should be going");
                    telemetry.update();
                }

                if (gamepad1.y && !gamepad1.yWasPressed()){
                    servo.setPower(-1);
                    telemetry.addLine("Should be going back");
                    telemetry.update();
                }

                if (gamepad1.b && !gamepad1.bWasPressed()){
                    servo.setPower(0);
                    telemetry.addLine("Should be going back");
                    telemetry.update();
                }

            }
        }
    }
}
