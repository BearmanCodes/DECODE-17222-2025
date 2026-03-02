package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Laser {
    private DigitalChannel laserSensor;
    private Telemetry telemetry;

    private boolean canAddBall = true;

    public Laser(HardwareMap hwmap, Telemetry telemetry){
        this.laserSensor = hwmap.get(DigitalChannel.class, "laser");
        this.telemetry = telemetry;
        this.laserSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isBallDetected(){
        boolean detected = laserSensor.getState();
        if (detected && canAddBall){
            canAddBall = false;
            return true;
        }
        if (!detected && !canAddBall){
            canAddBall = true;
            return false;
        }
        return false;
    }
}
