package org.firstinspires.ftc.teamcode.Temporary;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class ModeCore {

    public static double platformHeight;

    public static int flySpeed;

    public static boolean canMakeShot = false;

    public static double servoPosition;

    public static double LUIGI_HOPPER_LOAD = 0.15;

    public static double LUIGI_HOPPER_SHOOT = 0.47;

    public static double LUIGI_INTAKE_LOAD = 0.47;

    public static double HOPPER_LOAD_PLATFORM_HEIGHT = 0.4;

    public static double INTAKE_LOAD_PLATFORM_HEIGHT = 0.5;

    public static enum ALLIANCE {
        BLUE,
        RED
    }

    public static enum ROBOTS_SHOOTING_LOCATION {
        LINE_CLOSE,
        IN_CLOSE,
        LINE_FAR,
        IN_FAR,
        LEFT_FAR,
        RIGHT_FAR
    }

    public static enum BALL_DELIVERY_METHOD {
        HOPPER,
        INTAKE
    }

    public static enum DRIVE_MODE {
        MANUAL_DRIVE,
        AUTOMATED_DRIVE,
        SHOOT_MODE
    }
    private static Pose BLUE_HOPPER_LINECLOSE = new Pose(56.515188335358445, 88, Math.toRadians(135));

    private static Pose RED_HOPPER_LINECLOSE = new Pose(88.5, 88, Math.toRadians(45));


    public ALLIANCE currentAlliance;

    public static ROBOTS_SHOOTING_LOCATION desiredLocation;

    public static BALL_DELIVERY_METHOD deliveryCurrentMethod = BALL_DELIVERY_METHOD.HOPPER;

    public static DRIVE_MODE currentDriveMode = DRIVE_MODE.MANUAL_DRIVE;

    public static void autoShootHandler(Gamepad gamepad2, ALLIANCE currentAlliance){
        if (gamepad2.squareWasPressed()){
            desiredLocation = ROBOTS_SHOOTING_LOCATION.LINE_CLOSE;
            determineShotVariables(currentAlliance, deliveryCurrentMethod, desiredLocation);
        }
    }

    public static void determineShotVariables(ALLIANCE alliance, BALL_DELIVERY_METHOD deliveryMethod, ROBOTS_SHOOTING_LOCATION location){
        switch (alliance){
            case BLUE:
                switch (deliveryMethod){
                    case HOPPER:
                        switch (location){
                            case IN_FAR:
                                break;
                            case IN_CLOSE:
                                break;
                            case LEFT_FAR:
                                break;
                            case LINE_FAR:
                                break;
                            case RIGHT_FAR:
                                break;
                            case LINE_CLOSE:
                                platformHeight = 0.53;
                                flySpeed = 900;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_LINECLOSE;
                                break;
                        }
                        break;
                    case INTAKE:
                        switch (location){
                            case IN_FAR:
                                break;
                            case RIGHT_FAR:
                                break;
                            case LINE_FAR:
                                break;
                            case LEFT_FAR:
                                break;
                            case IN_CLOSE:
                                break;
                            case LINE_CLOSE:
                                break;
                        }
                        break;
                }
                break;
            case RED:
                switch (deliveryMethod){
                    case HOPPER:
                        switch (location){
                            case IN_FAR:
                                break;
                            case IN_CLOSE:
                                break;
                            case LEFT_FAR:
                                break;
                            case LINE_FAR:
                                break;
                            case RIGHT_FAR:
                                break;
                            case LINE_CLOSE:
                                platformHeight = 0.53;
                                flySpeed = 900;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_LINECLOSE;
                                break;
                        }
                        break;
                    case INTAKE:
                        break;
                }
                break;
        }
    }

    public static void prepareForShot(double platformHeight, int flySpeed, double servoPosition){
        TempShooterAutoCore.setLauncherPos(platformHeight);
        TempShooterAutoCore.spinUpFlys(flySpeed, flySpeed);
        TempShooterAutoCore.luigiServo.setPosition(servoPosition);
        canMakeShot = true;
    }
}
