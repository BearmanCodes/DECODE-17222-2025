package org.firstinspires.ftc.teamcode.Op;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Lights.Prism.Color;
import org.firstinspires.ftc.teamcode.Lights.Prism.Direction;
import org.firstinspires.ftc.teamcode.Lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Lights.Prism.PrismAnimations;

public class PrismCore {
    GoBildaPrismDriver prism;
    PrismAnimations.Snakes SUCK_SNAKE = new PrismAnimations.Snakes();

    PrismAnimations.Snakes SPIT_SNAKE = new PrismAnimations.Snakes();

    PrismAnimations.Solid INTAKE_NONE = new PrismAnimations.Solid(new Color(0, 0, 0));

    PrismAnimations.Solid LL_RED = new PrismAnimations.Solid(new Color(255, 255, 255));

    PrismAnimations.Solid LL_GOOD = new PrismAnimations.Solid(new Color(255, 0, 255));

    PrismAnimations.Solid LL_NONE = new PrismAnimations.Solid(new Color(0, 0, 0));

    PrismAnimations.Solid BAR_LIGHT = new PrismAnimations.Solid(new Color(255, 255, 255));


    public void Init(HardwareMap hwmap){
        prism = hwmap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(29);

        SUCK_SNAKE.setStartIndex(0);
        SUCK_SNAKE.setStopIndex(5);
        SUCK_SNAKE.setBrightness(100);
        SUCK_SNAKE.setSnakeLength(5);
        SUCK_SNAKE.setDirection(Direction.Forward);
        SUCK_SNAKE.setSpeed(0.3f);
        SUCK_SNAKE.setColors(new Color(0, 255, 0));

        SPIT_SNAKE.setStartIndex(0);
        SPIT_SNAKE.setStopIndex(5);
        SPIT_SNAKE.setBrightness(100);
        SPIT_SNAKE.setSnakeLength(5);
        SPIT_SNAKE.setDirection(Direction.Backward);
        SPIT_SNAKE.setSpeed(0.3f);
        SPIT_SNAKE.setColors(new Color(255, 0, 0));

        INTAKE_NONE.setStartIndex(0);
        INTAKE_NONE.setStopIndex(5);
        INTAKE_NONE.setBrightness(0);

        LL_RED.setStartIndex(6);
        LL_RED.setStopIndex(17);
        LL_RED.setBrightness(100);

        LL_GOOD.setStartIndex(6);
        LL_GOOD.setStopIndex(17);
        LL_GOOD.setBrightness(100);

        LL_NONE.setStartIndex(6);
        LL_NONE.setStopIndex(17);
        LL_NONE.setBrightness(0);

        BAR_LIGHT.setStartIndex(18);
        BAR_LIGHT.setStopIndex(29);
        BAR_LIGHT.setBrightness(100);
    }

    public void INTAKE_SPIT(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, SPIT_SNAKE);
    }

    public void BAR_LIGHT(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, BAR_LIGHT);
    }

    public void INTAKE_SUCK(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, SUCK_SNAKE);
    }

    public void INTAKE_NONE(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, INTAKE_NONE);
    }

    public void LL_BAD(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, LL_RED);
    }

    public void LL_GOOD(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, LL_GOOD);
    }

    public void LL_NONE(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, LL_NONE);
    }

    public void CLEAR(){
       prism.clearAllAnimations();
    }
}
