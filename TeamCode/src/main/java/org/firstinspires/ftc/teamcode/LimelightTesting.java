package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Temporary.PoseStorage;
import org.firstinspires.ftc.teamcode.Temporary.TempShooterAutoCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class LimelightTesting extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); //BLUE
    //new Pose(88.0744186, 8.037209302325575, Math.toRadians(0)); //RED
    public static Pose startingPose;

    public static double INITIAL_HEADING_OFFSET = 0;

    public static double SECOND_HEADING_OFFSET = 0;

    public static boolean useDefaultPose = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        TempShooterAutoCore.init(hardwareMap);
        follower.update();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        Pose pedroPose = follower.getPose();

        LLResult llResult = limelight.getLatestResult();
        limelight.updateRobotOrientation(Math.toDegrees(pedroPose.getHeading()) + INITIAL_HEADING_OFFSET); //Heading currently in degrees, convert to radians if necessary

        if (llResult != null && llResult.isValid()) {
            Pose3D threeDimensionalPose = llResult.getBotpose_MT2();
            Pose2D llPose = new Pose2D(DistanceUnit.METER, threeDimensionalPose.getPosition().x, threeDimensionalPose.getPosition().y, AngleUnit.DEGREES, threeDimensionalPose.getOrientation().getYaw() + SECOND_HEADING_OFFSET);
            Pose llToFTCPose = PoseConverter.pose2DToPose(llPose, InvertedFTCCoordinates.INSTANCE);
            Pose fieldPose = llToFTCPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            telemetry.addData("LL X: ", llPose.getX(DistanceUnit.METER));
            telemetry.addData("LL Y: ", llPose.getY(DistanceUnit.METER));
            telemetry.addData("LL HEADING: ", llPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("LLFTC X: ", llToFTCPose.getX());
            telemetry.addData("LLFTC Y: ", llToFTCPose.getY());
            telemetry.addData("LLFTC HEADING: ", llToFTCPose.getHeading());
            telemetry.addData("FIELD X: ", fieldPose.getX());
            telemetry.addData("FIELD Y: ", fieldPose.getY());
            telemetry.addData("FIELD HEADING: ", fieldPose.getHeading());
            telemetry.update();
        }
    }
}
