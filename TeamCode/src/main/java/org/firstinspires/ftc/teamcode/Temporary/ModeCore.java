package org.firstinspires.ftc.teamcode.Temporary;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class ModeCore {


    //ENUMS
    public static enum ALLIANCE {
        BLUE,
        RED
    }

    public static enum ROBOTS_SHOOTING_LOCATION {
        LINE_CLOSE,
        LEFT_FAR,
        RIGHT_FAR
    }

    public static enum DRIVE_MODE {
        MANUAL_DRIVE,
        AUTOMATED_DRIVE,
        SHOOT_MODE
    }
    //END ENUMS



    //SHARED VARIABLES
    public static double platformHeight;

    public static int flySpeed, frySpeed;
    public static boolean canMakeShot = false;

    //END SHARED VARIABLES


    //LUIGI POS
    public static double LUIGI_HOPPER_LOAD = 0.065;

    public static double LUIGI_HOPPER_SHOOT = 0.35;

    //END LUIGI POS


    //LAUNCHER POS
    public static double HOPPER_LOAD_PLATFORM_HEIGHT = 0.32;

    public static double BLUE_LEFT_FAR_LAUNCHER = 0.343; //changed name
    public static double RED_LEFT_FAR_LAUNCHER = 0.33;

    public static double BLUE_RIGHT_FAR_LAUNCHER = 0.47;
    public static double RED_RIGHT_FAR_LAUNCHER = 0.35;

    public static double BLUE_LINE_CLOSE_LAUNCHER = 0.53;
    public static double RED_LINE_CLOSE_LAUNCHER = 0.52; //0.53

    //END LAUNCHER POS

    //VELS
    public static int BLUE_LEFT_FAR_VEL = 1000;
    public static int BLUE_LEFT_FAR_VER = 1200;
    public static int RED_LEFT_FAR_VEL = 1000;
    public static int RED_LEFT_FAR_VER = 1000;

    public static int BLUE_RIGHT_FAR_VEL = 1000;
    public static int BLUE_RIGHT_FAR_VER = 1000;
    public static int RED_RIGHT_FAR_VEL = 975;
    public static int RED_RIGHT_FAR_VER = 1175;

    public static int BLUE_LINE_CLOSE_VEL = 850;
    public static int BLUE_LINE_CLOSE_VER = 875;
    public static int RED_LINE_CLOSE_VEL = 875;
    public static int RED_LINE_CLOSE_VER = 900;

    //END VELS
    public static ROBOTS_SHOOTING_LOCATION desiredLocation;

    public static DRIVE_MODE currentDriveMode = DRIVE_MODE.MANUAL_DRIVE;

    public static void autoShootHandler(Gamepad gamepad2, ALLIANCE currentAlliance) {
        if (gamepad2.squareWasPressed()) {
            desiredLocation = ROBOTS_SHOOTING_LOCATION.LINE_CLOSE;
            determineShotVariables(currentAlliance, desiredLocation);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            desiredLocation = ROBOTS_SHOOTING_LOCATION.LEFT_FAR;
            determineShotVariables(currentAlliance, desiredLocation);
        }
        if (gamepad2.dpadRightWasPressed()) {
            desiredLocation = ROBOTS_SHOOTING_LOCATION.RIGHT_FAR;
            determineShotVariables(currentAlliance, desiredLocation);
        }
    }

    public static void determineShotVariables(ALLIANCE alliance, ROBOTS_SHOOTING_LOCATION location) {
        switch (alliance) {
            case BLUE:
                switch (location) {
                    case LEFT_FAR:
                        platformHeight = BLUE_LEFT_FAR_LAUNCHER;
                        flySpeed = BLUE_LEFT_FAR_VEL;
                        frySpeed = BLUE_LEFT_FAR_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_BLUE.targetPose = PoseCore.BLUE_LEFT_FAR_POSE;
                        break;
                    case RIGHT_FAR:
                        platformHeight = BLUE_RIGHT_FAR_LAUNCHER;
                        flySpeed = BLUE_RIGHT_FAR_VEL;
                        frySpeed = BLUE_RIGHT_FAR_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_BLUE.targetPose = PoseCore.BLUE_RIGHT_FAR_POSE;
                        break;
                    case LINE_CLOSE:
                        platformHeight = BLUE_LINE_CLOSE_LAUNCHER;
                        flySpeed = BLUE_LINE_CLOSE_VEL;
                        frySpeed = BLUE_LINE_CLOSE_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_BLUE.targetPose = PoseCore.BLUE_LINE_CLOSE_POSE;
                        break;
                }
                break;
            case RED:
                switch (location) {
                    case RIGHT_FAR:
                        platformHeight = RED_RIGHT_FAR_LAUNCHER;
                        flySpeed = RED_RIGHT_FAR_VEL;
                        frySpeed = RED_RIGHT_FAR_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_RED.targetPose = PoseCore.RED_RIGHT_FAR_POSE;
                        break;
                    case LEFT_FAR:
                        platformHeight = RED_LEFT_FAR_LAUNCHER;
                        flySpeed = RED_LEFT_FAR_VEL;
                        frySpeed = RED_RIGHT_FAR_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_RED.targetPose = PoseCore.RED_LEFT_FAR_POSE;
                        break;
                    case LINE_CLOSE:
                        platformHeight = RED_LINE_CLOSE_LAUNCHER;
                        flySpeed = RED_LINE_CLOSE_VEL;
                        frySpeed = RED_LINE_CLOSE_VER;
                        prepareForShot(platformHeight, flySpeed, frySpeed);
                        AutoTeleOp_RED.targetPose = PoseCore.RED_LINE_CLOSE_POSE;
                        break;
                }
                break;
        }
    }
    public static void prepareForShot(double platformHeight, int flySpeed, int frySpeed) {
        TempShooterAutoCore.setLauncherPos(platformHeight);
        TempShooterAutoCore.spinUpFlys(flySpeed, frySpeed);
        canMakeShot = true;
    }
}
