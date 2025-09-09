package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadCore {
    public static Gamepad gpad1;
    public static Gamepad gpad2;

    public static Gamepad currGamepad1 = new Gamepad();
    public static Gamepad prevGamepad1 = new Gamepad();

    public static Gamepad currGamepad2 = new Gamepad();
    public static Gamepad prevGamepad2 = new Gamepad();
    public static void edgeDetector() {
        prevGamepad1.copy(currGamepad1);
        currGamepad1.copy(gpad1);
        prevGamepad2.copy(currGamepad2);
        currGamepad2.copy(gpad2);
    }

    public static boolean riseEdge(boolean curr, boolean prev){
        return curr && !prev;
    }
}
