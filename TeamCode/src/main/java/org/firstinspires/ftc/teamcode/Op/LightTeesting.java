package org.firstinspires.ftc.teamcode.Op;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lights.Prism.GoBildaPrismDriver;

@TeleOp
public class LightTeesting extends OpMode {
    PrismCore prismCore = new PrismCore();
    @Override
    public void init() {
        prismCore.Init(hardwareMap);
    }

    @Override
    public void stop(){
        prismCore.prism.clearAllAnimations();
    }

    @Override
    public void start(){
        prismCore.INTAKE_SPIT();
        prismCore.BAR_LIGHT();
        prismCore.LL_GOOD();
    }

    @Override
    public void loop() {
        //prismCore.LL_GOOD();
        //prismCore.BAR_LIGHT();
    }
}
