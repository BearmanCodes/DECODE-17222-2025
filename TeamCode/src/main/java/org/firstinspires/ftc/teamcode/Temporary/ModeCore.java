package org.firstinspires.ftc.teamcode.Temporary;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class ModeCore {

    public static double BLUE_HOPPER_RIGHT_FAR_X = 84;
    public static double BLUE_HOPPER_RIGHT_FAR_Y = 11.5;

    public static double BLUE_HOPPER_RIGHT_FAR_HEADING = 125;

    private static final Pose BLUE_HOPPER_RIGHT_FAR = new Pose(BLUE_HOPPER_RIGHT_FAR_X, BLUE_HOPPER_RIGHT_FAR_Y, Math.toRadians(BLUE_HOPPER_RIGHT_FAR_HEADING));
    public static double BLUE_HOPPER_LINE_FAR_X = 96;
    public static double BLUE_HOPPER_LINE_FAR_Y = 128;

    public static double BLUE_HOPPER_LINE_FAR_HEADING = 180;

    private static final Pose BLUE_HOPPER_LINE_FAR = new Pose(BLUE_HOPPER_LINE_FAR_X, BLUE_HOPPER_LINE_FAR_Y, Math.toRadians(BLUE_HOPPER_LINE_FAR_HEADING));
    public static double BLUE_HOPPER_IN_FAR_X = 96;
    public static double BLUE_HOPPER_IN_FAR_Y = 128;

    public static double BLUE_HOPPER_IN_FAR_HEADING = 180;

    private static Pose BLUE_HOPPER_IN_FAR = new Pose(BLUE_HOPPER_IN_FAR_X, BLUE_HOPPER_IN_FAR_Y, Math.toRadians(BLUE_HOPPER_IN_FAR_HEADING));

    public static double BLUE_HOPPER_IN_CLOSE_X = 57;
    public static double BLUE_HOPPER_IN_CLOSE_Y = 128;

    public static double BLUE_HOPPER_IN_CLOSE_HEADING = 180;
    private static Pose BLUE_HOPPER_IN_CLOSE = new Pose(BLUE_HOPPER_IN_CLOSE_X, BLUE_HOPPER_IN_CLOSE_Y, Math.toRadians(BLUE_HOPPER_IN_FAR_HEADING));

    public static double platformHeight;

    public static int flySpeed;

    public static boolean canMakeShot = false;

    public static double servoPosition;

    public static double LUIGI_HOPPER_LOAD = 0.15;

    public static double LUIGI_HOPPER_SHOOT = 0.47;

    public static double LUIGI_INTAKE_LOAD = 0.47;

    public static double HOPPER_LOAD_PLATFORM_HEIGHT = 0.4;

    public static double INTAKE_LOAD_PLATFORM_HEIGHT = 0.5;

    public static double RED_HOPPER_LINE_CLOSE_LAUNCHER = 0.52; //0.53
    public static int RED_HOPPER_LINE_CLOSE_VELS = 900; //900

    public static double RED_INTAKE_LINE_CLOSE_LAUNCHER = 0.52; //0.53
    public static int RED_INTAKE_LINE_CLOSE_VELS = 800; //900
    public static double RED_INTAKE_LINE_CLOSE_SERVO = 0.47;


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

    public static double RED_HOPPER_LEFT_FAR_LAUNCHER = 0.47;
    public static int RED_HOPPER_LEFT_FAR_VELS = 1000;

    public static double RED_HOPPER_LEFT_FAR_X = 88.10091747889908;
    public static double RED_HOPPER_LEFT_FAR_Y = 13.10091743119265;
    public static double RED_HOPPER_LEFT_FAR_Heading = 73;
    private static Pose RED_HOPPER_LEFT_FAR = new Pose(RED_HOPPER_LEFT_FAR_X, RED_HOPPER_LEFT_FAR_Y, Math.toRadians(RED_HOPPER_LEFT_FAR_Heading));

    public static double RED_HOPPER_RIGHT_FAR_LAUNCHER = 0.47;
    public static int RED_HOPPER_RIGHT_FAR_VELS = 1000;

    public static double BLUE_HOPPER_FAR_LEFT_X = 62;
    public static double BLUE_HOPPER_FAR_LEFT_Y = 11.5;
    public static double BLUE_HOPPER_FAR_LEFT_Heading = 105;

    private Pose RED_HOPPER_RIGHT_FAR = new Pose(RED_HOPPER_FAR_RIGHT_X, RED_HOPPER_FAR_RIGHT_Y, Math.toRadians(RED_HOPPER_FAR_RIGHT_Heading));
    public static double RED_HOPPER_FAR_RIGHT_X = 84;
    public static double RED_HOPPER_FAR_RIGHT_Y = 11.5;
    public static double RED_HOPPER_FAR_RIGHT_Heading = 65;

    public static int RED_HOPPER_FAR_RIGHT_VELS = 1000;

    public static int BLUE_HOPPER_FAR_LEFT_VELS = 1000;

    public static int BLUE_HOPPER_IN_FAR_VELS = 950;

    public static int BLUE_HOPPER_IN_CLOSE_VELS = 750;

    public static int BLUE_HOPPER_LINE_FAR_VELS = 950;

    public static int BLUE_HOPPER_RIGHT_FAR_VELS = 1000;

    public static int BLUE_HOPPER_LINE_CLOSE_VELS = 900;

    public static double RED_HOPPER_FAR_RIGHT_LAUNCHER = 0.45;
    public static double BLUE_HOPPER_FAR_LEFT_LAUNCHER = 0.45;


    public static double BLUE_HOPPER_IN_FAR_LAUNCHER = 0.45;

    public static double BLUE_HOPPER_IN_CLOSE_LAUNCHER = 0.53;

    public static double BLUE_HOPPER_LINE_FAR_LAUNCHER = 0.45;

    public static double BLUE_HOPPER_RIGHT_FAR_LAUNCHER = 0.47;
    public static  double BLUE_INTAKE_RIGHT_FAR_LAUNCHER = 0.47;

    public static double RED_INTAKE_RIGHT_FAR_LAUNCHER = 0.47;
    public static int RED_INTAKE_RIGHT_FAR_VELS = 1000;

    public static double BLUE_HOPPER_LINE_CLOSE_LAUNCHER = 0.53;

    public static double RED_INTAKE_RIGHT_FAR_SERVO = 0.45;

    public static  double BLUE_INTAKE_RIGHT_FAR_SERVO = 0.47;

    public static int BLUE_INTAKE_RIGHT_FAR_VELS = 1000;

    public static double BLUE_INTAKE_LEFT_FAR_LAUNCHER = 0.47;
    public static int BLUE_INTAKE_LEFT_FAR_VELS = 1000;
    public static double BLUE_INTAKE_LEFT_FAR_SERVO = 0.45;

    public static double RED_INTAKE_LEFT_FAR_LAUNCHER = 0.47;
    public static int RED_INTAKE_LEFT_FAR_VELS = 1000;
    public static double RED_INTAKE_LEFT_FAR_SERVO = 0.47;

    public static double BLUE_INTAKE_LINE_CLOSE_LAUNCHER = 0.53;
    public static int BLUE_INTAKE_LINE_CLOSE_VELS = 800;
    public static double BLUE_INTAKE_LINE_CLOSE_SERVO = 0.47;



    public static double BLUE_INTAKE_FAR_LEFT_SERVO = 0.47;
    private static Pose RED_HOPPER_FAR_RIGHT = new Pose(RED_HOPPER_FAR_RIGHT_X, RED_HOPPER_FAR_RIGHT_Y, Math.toRadians(RED_HOPPER_FAR_RIGHT_Heading));


    private static Pose BLUE_HOPPER_FAR_LEFT = new Pose(BLUE_HOPPER_FAR_LEFT_X, BLUE_HOPPER_FAR_LEFT_Y, Math.toRadians(BLUE_HOPPER_FAR_LEFT_Heading));

    private static Pose BLUE_HOPPER_LINECLOSE = new Pose(56.515188335358445, 88, Math.toRadians(135));

    private static Pose RED_HOPPER_LINECLOSE = new Pose(90, 85, Math.toRadians(45));
    public ALLIANCE currentAlliance;

    public static ROBOTS_SHOOTING_LOCATION desiredLocation;

    public static BALL_DELIVERY_METHOD deliveryCurrentMethod = BALL_DELIVERY_METHOD.HOPPER;

    public static DRIVE_MODE currentDriveMode = DRIVE_MODE.MANUAL_DRIVE;

    public static void autoShootHandler(Gamepad gamepad2, ALLIANCE currentAlliance){
        if (gamepad2.squareWasPressed()){
            desiredLocation = ROBOTS_SHOOTING_LOCATION.LINE_CLOSE;
            determineShotVariables(currentAlliance, deliveryCurrentMethod, desiredLocation);
        }
        if (gamepad2.dpadLeftWasPressed()){
            desiredLocation = ROBOTS_SHOOTING_LOCATION.LEFT_FAR;
            determineShotVariables(currentAlliance, deliveryCurrentMethod, desiredLocation);
        }
        if (gamepad2.dpadRightWasPressed()){
            desiredLocation = ROBOTS_SHOOTING_LOCATION.RIGHT_FAR;
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
                                platformHeight = BLUE_HOPPER_IN_FAR_LAUNCHER;
                                flySpeed = BLUE_HOPPER_IN_FAR_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_IN_FAR;
                                break;
                            case IN_CLOSE:
                                platformHeight = BLUE_HOPPER_IN_CLOSE_LAUNCHER;
                                flySpeed = BLUE_HOPPER_IN_CLOSE_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_IN_CLOSE;
                                break;
                            case LEFT_FAR:
                                platformHeight = BLUE_HOPPER_FAR_LEFT_LAUNCHER;
                                flySpeed = BLUE_HOPPER_FAR_LEFT_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_FAR_LEFT;
                                break;
                            case LINE_FAR:
                                platformHeight = BLUE_HOPPER_LINE_FAR_LAUNCHER;
                                flySpeed = BLUE_HOPPER_LINE_FAR_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_LINE_FAR;
                                break;
                            case RIGHT_FAR:
                                platformHeight = BLUE_HOPPER_RIGHT_FAR_LAUNCHER;
                                flySpeed = BLUE_HOPPER_RIGHT_FAR_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_RIGHT_FAR;
                                break;
                            case LINE_CLOSE:
                                platformHeight = BLUE_HOPPER_LINE_CLOSE_LAUNCHER;
                                flySpeed = BLUE_HOPPER_LINE_CLOSE_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_LINECLOSE;
                                break;
                        }
                        break;
                    case INTAKE:
                        switch (location){
                            case IN_FAR:
                               // platformHeight = BLUE_INTAKE_IN_FAR_LAUNCHER;
                               // flySpeed = BLUE_INTAKE_IN_FAR_VELS;
                               // servoPosition = BLUE_INTAKE_IN_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_IN_FAR;
                                break;
                            case RIGHT_FAR:
                                platformHeight = BLUE_INTAKE_RIGHT_FAR_LAUNCHER;
                                flySpeed = BLUE_INTAKE_RIGHT_FAR_VELS;
                                servoPosition = BLUE_INTAKE_RIGHT_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_RIGHT_FAR;
                                break;
                            case LINE_FAR:
                                //platformHeight = BLUE_INTAKE_LINE_FAR_LAUNCHER;
                                //flySpeed = BLUE_INTAKE_LINE_FAR_VELS;
                                //servoPosition = BLUE_INTAKE_LINE_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_LINE_FAR;
                                break;
                            case LEFT_FAR:
                                platformHeight = BLUE_INTAKE_LEFT_FAR_LAUNCHER;
                                flySpeed = BLUE_INTAKE_LEFT_FAR_VELS;
                                servoPosition = BLUE_INTAKE_LEFT_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_FAR_LEFT;
                                break;
                            case IN_CLOSE:
                                //platformHeight = BLUE_INTAKE_IN_CLOSE_LAUNCHER;
                                //flySpeed = BLUE_INTAKE_IN_CLOSE_VELS;
                                //servoPosition = BLUE_INTAKE_IN_CLOSE_SERVO;
                                //prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_BLUE.targetPose = BLUE_IN_CLOSE;
                                break;
                            case LINE_CLOSE:
                                platformHeight = BLUE_INTAKE_LINE_CLOSE_LAUNCHER;
                                flySpeed = BLUE_INTAKE_LINE_CLOSE_VELS;
                                servoPosition = BLUE_INTAKE_LINE_CLOSE_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_BLUE.targetPose = BLUE_HOPPER_LINECLOSE;
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
                                //platformHeight = RED_HOPPER_IN_FAR_LAUNCHER;
                                //flySpeed = RED_HOPPER_IN_FAR_VELS;
                                //servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_HOPPER_IN_FAR;
                                break;
                            case IN_CLOSE:
                                //platformHeight = RED_HOPPER_IN_CLOSE_LAUNCHER;
                                //flySpeed = RED_HOPPER_IN_CLOSE_VELS;
                                //servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_HOPPER_IN_CLOSE;
                                break;
                            case LEFT_FAR:
                                platformHeight = RED_HOPPER_LEFT_FAR_LAUNCHER;
                                flySpeed = RED_HOPPER_LEFT_FAR_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_LEFT_FAR;
                                break;
                            case LINE_FAR:
                                //platformHeight = RED_HOPPER_LINE_FAR_LAUNCHER;
                                //flySpeed = RED_HOPPER_LINE_FAR_VELS;
                                //servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_HOPPER_LINE_FAR;
                                break;
                            case RIGHT_FAR:
                                platformHeight = RED_HOPPER_RIGHT_FAR_LAUNCHER;
                                flySpeed = RED_HOPPER_RIGHT_FAR_VELS;
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_FAR_RIGHT;
                                break;
                            case LINE_CLOSE:
                                platformHeight = RED_HOPPER_LINE_CLOSE_LAUNCHER; //0.53
                                flySpeed = RED_HOPPER_LINE_CLOSE_VELS; //900
                                servoPosition = LUIGI_HOPPER_LOAD;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_LINECLOSE;
                                break;
                        }
                        break;
                    case INTAKE:
                        switch (location){
                            case IN_FAR:
                                //platformHeight = RED_INTAKE_IN_FAR_LAUNCHER;
                                //flySpeed = RED_INTAKE_IN_FAR_VELS;
                                //servoPosition = RED_INTAKE_IN_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_INTAKE_IN_FAR;
                                break;
                            case IN_CLOSE:
                                //platformHeight = RED_INTAKE_IN_CLOSE_LAUNCHER;
                                //flySpeed = RED_INTAKE_IN_CLOSE_VELS;
                                //servoPosition = RED_INTAKE_IN_CLOSE_VELS;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_INTAKE_IN_CLOSE;
                                break;
                            case LEFT_FAR:
                                platformHeight = RED_INTAKE_LEFT_FAR_LAUNCHER;
                                flySpeed = RED_INTAKE_LEFT_FAR_VELS;
                                servoPosition = RED_INTAKE_LEFT_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_LEFT_FAR;
                                break;
                            case LINE_FAR:
                                //platformHeight = RED_INTAKE_LINE_FAR_LAUNCHER;
                                //flySpeed = RED_INTAKE_LINE_FAR_VELS;
                                //servoPosition = RED_INTAKE_LINE_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                //AutoTeleOp_RED.targetPose = RED_INTAKE_LINE_FAR;
                                break;
                            case RIGHT_FAR:
                                platformHeight = RED_INTAKE_RIGHT_FAR_LAUNCHER;
                                flySpeed = RED_INTAKE_RIGHT_FAR_VELS;
                                servoPosition = RED_INTAKE_RIGHT_FAR_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_FAR_RIGHT;
                                break;
                            case LINE_CLOSE:
                                platformHeight = RED_INTAKE_LINE_CLOSE_LAUNCHER; //0.53
                                flySpeed = RED_INTAKE_LINE_CLOSE_VELS; //900
                                servoPosition = RED_INTAKE_LINE_CLOSE_SERVO;
                                prepareForShot(platformHeight, flySpeed, servoPosition);
                                AutoTeleOp_RED.targetPose = RED_HOPPER_LINECLOSE;
                                break;
                        }
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
