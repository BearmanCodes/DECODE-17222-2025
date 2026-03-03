package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Kickstand {
    double init_pos = 0.0;
    public static double extended_pos = 0.31;

    Telemetry telemetry;

    Gamepad gamepad;

    Servo kickstand;

    public Kickstand(HardwareMap hwMap, Telemetry _telemetry, Gamepad _gamepad) {
        this.telemetry = _telemetry;
        this.gamepad = _gamepad;
        this.kickstand = hwMap.get(Servo.class, "kicker");
        this.kickstand.setDirection(Servo.Direction.REVERSE);
        this.kickstand.setPosition(init_pos);
    }

    public void handler(){
        if (gamepad.rightStickButtonWasPressed()){
            kickstand.setPosition(extended_pos);
        }
        if (gamepad.shareWasPressed()) {
            kickstand.setPosition(init_pos);
        }
    }
}
