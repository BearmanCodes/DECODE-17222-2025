package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Objects;

public class LimelightCore {
    public static Limelight3A limelight;
    public static double limelightMountAngleDegrees = 0;
    public static double limelightLensHeightInches = 15.75;
    public static double goalHeightInches = 32;

    /*
    public static double targetOffsetAngle_Vertical = Math.round(llResult.getTy() * 100.00) / 100.00;
    public static double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    public static double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    public static double distanceFromLimelightToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    */

    public static Pose3D mt2_update(){
        return Objects.requireNonNull(get_result()).getBotpose_MT2();
    }

    public static LLResult get_result() {
        LLResult llResult = limelight.getLatestResult();

        

        if (llResult != null && llResult.isValid()) {
            return get_result();
        }
        return null;
    }

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }
}
