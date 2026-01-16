package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class LeviTest extends LinearOpMode {
    public DcMotorEx fly, fry;

    public static double FLY_VEL = 900;

    public static double FRY_VEL = 900;

    public static boolean FLY_FWD = false;

    public static boolean FRY_FWD = true;

    @Override
    public void runOpMode() throws InterruptedException {
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fry = hardwareMap.get(DcMotorEx.class, "fry");
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorSimple.Direction flyDir = FLY_FWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction fryDir = FRY_FWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        fly.setDirection(flyDir);
        fry.setDirection(fryDir);
        waitForStart();
        while (!isStopRequested()) {
            fly.setVelocity(FLY_VEL);
            fry.setVelocity(FRY_VEL);
        }
    }
}
