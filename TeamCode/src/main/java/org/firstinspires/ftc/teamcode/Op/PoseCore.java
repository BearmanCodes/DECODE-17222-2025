package org.firstinspires.ftc.teamcode.Op;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class PoseCore {
    //RIGHT_FAR
    public static double BLUE_RIGHT_FAR_X = 84;
    public static double BLUE_RIGHT_FAR_Y = 11.5;
    public static double BLUE_RIGHT_FAR_HEADING = 125;
    public static final Pose BLUE_RIGHT_FAR_POSE = new Pose(BLUE_RIGHT_FAR_X, BLUE_RIGHT_FAR_Y, Math.toRadians(BLUE_RIGHT_FAR_HEADING));


    public static double RED_RIGHT_FAR_X = 86.5;
    public static double RED_RIGHT_FAR_Y = 15.0;
    public static double RED_RIGHT_FAR_HEADING = 67.5;
    public static Pose RED_RIGHT_FAR_POSE = new Pose(RED_RIGHT_FAR_X, RED_RIGHT_FAR_Y, Math.toRadians(RED_RIGHT_FAR_HEADING));

    //END RIGHT_FAR

    //LEFT_FAR
    public static double BLUE_LEFT_FAR_X = 61.5;
    public static double BLUE_LEFT_FAR_Y = 14.0;
    public static double BLUE_LEFT_FAR_HEADING = 112;
    public static Pose BLUE_LEFT_FAR_POSE = new Pose(BLUE_LEFT_FAR_X, BLUE_LEFT_FAR_Y, Math.toRadians(BLUE_LEFT_FAR_HEADING));


    public static double RED_LEFT_FAR_X = 88.10091747889908;
    public static double RED_LEFT_FAR_Y = 13.10091743119265;
    public static double RED_LEFT_FAR_HEADING = 73;
    public static Pose RED_LEFT_FAR_POSE = new Pose(RED_LEFT_FAR_X, RED_LEFT_FAR_Y, Math.toRadians(RED_LEFT_FAR_HEADING));

    //END LEFT_FAR

    //LINE_CLOSE

    public static double BLUE_LINE_CLOSE_X = 61;
    public static double BLUE_LINE_CLOSE_Y = 82;
    public static double BLUE_LINE_CLOSE_HEADING = 135.46;
    public static Pose BLUE_LINE_CLOSE_POSE = new Pose(BLUE_LINE_CLOSE_X, BLUE_LINE_CLOSE_Y, Math.toRadians(BLUE_LINE_CLOSE_HEADING));


    public static double RED_LINE_CLOSE_X = 83;
    public static double RED_LINE_CLOSE_Y = 82;
    public static double RED_LINE_CLOSE_HEADING = 46.15;
    public static Pose RED_LINE_CLOSE_POSE = new Pose(RED_LINE_CLOSE_X, RED_LINE_CLOSE_Y, Math.toRadians(RED_LINE_CLOSE_HEADING));

    //END LINE_CLOSE
}
