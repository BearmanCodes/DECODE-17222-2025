package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    static double ROBO_MASS = 0/*KILOGRAMS*/;
    static double MAX_POWER = 1;

    static DcMotorSimple.Direction FL_DIR = DcMotorSimple.Direction.FORWARD;
    static DcMotorSimple.Direction FR_DIR = DcMotorSimple.Direction.FORWARD;
    static DcMotorSimple.Direction BL_DIR = DcMotorSimple.Direction.FORWARD;
    static DcMotorSimple.Direction BR_DIR = DcMotorSimple.Direction.FORWARD;

    //Forward Pod (X) offset in the Y direction. Don't confuse the lettering it's weird.
    static double FPOD_Y_OFFSET = 0; //Inches but can adjust to whatever DistanceUnit is

    //Strafe Pod (Y) offset in the X direction. Don't confuse the lettering it's weird.
    static double SPOD_X_OFFSET = 0; //Inches but can adjust to whatever DistanceUnit is

    static DistanceUnit OFFSET_DISTANCE_UNIT = DistanceUnit.INCH;

    static String PINPOINT_IMU_NAME = "imu";

    static GoBildaPinpointDriver.EncoderDirection FPOD_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD; //Subject have to change

    static GoBildaPinpointDriver.EncoderDirection SPOD_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD; //Subject to change

    //AUTOMATIC TUNING CONSTANTS
    //FORWARD_VELOCITY_TUNER
    static double X_VELOCITY = 0;
    //LATERAL_VELOCITY_TUNER
    static double Y_VELOCITY = 0;
    //FORWARD_ZERO_POWER_ACCELERATION
    static double X_ZERO_POWER_ACCELERATION = 0;
    //LATERAL_ZERO_POWER_ACCELERATION
    static double Y_ZERO_POWER_ACCELERATION = 0;



    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(ROBO_MASS)
            .forwardZeroPowerAcceleration(X_ZERO_POWER_ACCELERATION)
            .lateralZeroPowerAcceleration(Y_ZERO_POWER_ACCELERATION)
            /* COEFFICENTS ONCE YOU TUNE TRANSITIONAL PIDF MANUTALLY .translationalPIDFCoefficients(new PIDFCoefficients())*/
            /* ONCE YOU TUNE HEADING PIDF MANUALLY .headingPIDFCoefficients(new PIDFCoefficients())*/
            /*ONCE YOU TUNE DRIVE PIDF, THIS WILL TAKE LIKE FOREVER, AHHH .drivePIDFCoefficients(new FilteredPIDFCoefficients())*/
            /*TUNE CENTRIPETAL SCALING .centripetalScaling() */;

    //Pinpoint Offsets Reference https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
    //NOT THE SAME AS FOR CLAIBRATING THE ODOPOD, DIFFERENT SYSTEM, NEEDS TO BE CENTER OF ROTATION
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(FPOD_Y_OFFSET)
            .strafePodX(SPOD_X_OFFSET)
            .distanceUnit(OFFSET_DISTANCE_UNIT)
            .hardwareMapName(PINPOINT_IMU_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(FPOD_ENCODER_DIRECTION)
            .strafeEncoderDirection(SPOD_ENCODER_DIRECTION);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(MAX_POWER)
            .leftFrontMotorName("frontleft")
            .rightFrontMotorName("frontright")
            .leftRearMotorName("backleft")
            .rightRearMotorName("backright")
            .leftFrontMotorDirection(FL_DIR)
            .rightFrontMotorDirection(FR_DIR)
            .leftRearMotorDirection(BL_DIR)
            .rightRearMotorDirection(BR_DIR)
            .xVelocity(X_VELOCITY)
            .yVelocity(Y_VELOCITY);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
