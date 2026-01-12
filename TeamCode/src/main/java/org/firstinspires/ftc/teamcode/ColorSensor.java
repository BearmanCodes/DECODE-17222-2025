package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor extends LinearOpMode {
    com.qualcomm.robotcore.hardware.ColorRangeSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorRangeSensor.class, "color");
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("red: ", color.red());
            telemetry.addData("blue: ", color.blue());
            telemetry.addData("green: ", color.green());
            telemetry.addData("Distance: ", color.getDistance(DistanceUnit.MM));
            telemetry.addData("argb: ", color.argb());
            telemetry.update();
        }
    }
}
