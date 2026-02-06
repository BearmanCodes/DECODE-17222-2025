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
    public static double INTIAL_HEADING_LL_OFFSET = -270;

    private Limelight3A limelight;
    public static Pose defaultStartingPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180 + INTIAL_HEADING_LL_OFFSET)); //BLUE
    //new Pose(88.0744186, 8.037209302325575, Math.toRadians(0)); //RED
    public static Pose startingPose;


    public static double MT2_HEADING_OFFSET = -90;

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
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        Pose pedroPose = follower.getPose();

        LLResult llResult = limelight.getLatestResult();
        limelight.updateRobotOrientation(Math.toDegrees(pedroPose.getHeading()) + MT2_HEADING_OFFSET); //Heading currently in degrees, convert to radians if necessary

        if (llResult != null && llResult.isValid()) {
            Pose3D threeDimensionalPose = llResult.getBotpose_MT2();
            Pose2D llPose = new Pose2D(DistanceUnit.METER, threeDimensionalPose.getPosition().x, threeDimensionalPose.getPosition().y, AngleUnit.DEGREES, threeDimensionalPose.getOrientation().getYaw());
            Pose llToFTCPose = PoseConverter.pose2DToPose(llPose, InvertedFTCCoordinates.INSTANCE);
            Pose fieldPose = llToFTCPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            telemetry.addData("Pose3D X: ", threeDimensionalPose.getPosition().x);
            telemetry.addData("Pose3D Y: ", threeDimensionalPose.getPosition().y);
            telemetry.addData("Pose3D HEADING: ", threeDimensionalPose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addData("LL X: ", llPose.getX(DistanceUnit.METER));
            telemetry.addData("LL Y: ", llPose.getY(DistanceUnit.METER));
            telemetry.addData("LL HEADING: ", llPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("LLFTC X: ", llToFTCPose.getX());
            telemetry.addData("LLFTC Y: ", llToFTCPose.getY());
            telemetry.addData("LLFTC HEADING: ", llToFTCPose.getHeading());
            //to convert to ftc, convert data from meters to inches
            
            //for rotation to appear correct on the limelight, the intial position of 180 had to become -90. Meaning it went from <- to ↓. So when the robot was <- the imu was reading a value of -90 deg. 
            //Then, the limelight was getting that value and subtracting another -90 degrees and the orientation was correct.
            //So, as far as the limelight is concerned, at <- the imu value should be -90 - 90 which is -180. Now pedro standard is to read a value of 180 at <-. Instead, limelight wanted that value to be -180, to my knowledge.
            //Therefore, I believe that limelight wants to be fed a negative value. 
            //This is not set in stone so work on this later, my brain is telling me that it should be fed a negative value -90 degrees so we'll see. 

            //Converting from ftc coordinates to pedro coordinates, FTC is in inches along with pedro so that's fine. 
            //Pedro flips the x and y axis between each other, so while going towards the blue secret tunnel increases x in pedro, it increases y in FTC.
            //On top of this, the y axis is opposite. So while going up towards the obelisk increases y in pedro, it decreases x in FTC.
            //So given a coordinate of (x,y) in FTC, the cooresponding pedro coordinate is (y, -x). 
            //Also, while FTC's origin is (0,0) Pedro's origin is (72, 72), meaning that you should add 72 to the coordinates after you swap -em
            //So the FTC coordinate of (-58.3727, 55.6425) is the pedro coordinate (55.6425 + 72, 58.3727 + 72) or (127.6425, 130.3727)
            //Therefore, the formula of (x, y) in FTC is (y + 72, -x + 72) in Pedro

            //Similarly, the rotation is shifted. In ftc, rotation follows 0: Facing audience, 90: Facing Blue Alliance (Red Goal), 180: Facing Refs/TV, 270: Facing Red Alliance (Blue Goal) [Which Ll auto corrected to -90?]
            //In pedro, rotation follows 0: Facing Blue Alliance (Red Goal), 90: Facing Refs/TV, 180: Facing Red Alliance (Blue Goal), 270: Facing audience.
            //Meaning, if things are simple, all you need to do to convert FTC to Pedro headings are to subtract 90 degrees or Pi/2 rads. BUt I have a feeling it's not so simple
            //The best thing to do is clamp the heading via
            /*
                if deg < 0:
                    deg += 360;
                else if deg >= 360:
                    deg -= 360;
            */
            /*
            public double clamp_heading_deg(double heading_deg){
                if (heading_deg < 0){
                    return heading_deg + 360;
                } 
                if (heading_deg >= 360) {
                    return heading_deg - 360;
                }
                return heading_deg;
            }
            public Pose2D ll_to_FTC(double ll_x, double ll_y, double ll_heading_deg){
                double ftc_x = ll_x * 39.37;
                double ftc_y = ll_y * 39.37;
                double ftc_heading_deg = clamp_heading_deg(ll_heading_deg);
                Pose2D ftcPose = new Pose2D(ftc_x, ftc_y, ftc_heading_deg);
                return ftcPose;
            }
            public double pedro_rad_to_MT2_heading_deg(double pedro_rad_heading){
                double pedro_deg_heading = Math.toDegrees(pedro_rad_heading);
                return clamp_heading_deg(pedro_deg_heading + 90);
            }
            public Pose FTC_to_Pedro(double FTC_x, double FTC_y, double FTC_heading_deg){
                double pedro_x = FTC_y + 72;
                double pedro_y = -FTC_x + 72;
                double pedro_heading_deg = clamp_heading_deg(FTC_heading_deg - 90);
                Pose pedroPose = new Pose(pedro_x, pedro_y, Math.toRadians(pedro_heading_deg));
                return pedroPose;
            }
            */
            telemetry.addData("FIELD X: ", fieldPose.getX());
            telemetry.addData("FIELD Y: ", fieldPose.getY());
            telemetry.addData("FIELD HEADING: ", fieldPose.getHeading());
            telemetry.update();
        }
    }
}
