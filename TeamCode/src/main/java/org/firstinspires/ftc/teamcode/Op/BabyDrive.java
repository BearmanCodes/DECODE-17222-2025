package org.firstinspires.ftc.teamcode.Op;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Kickstand;
import org.firstinspires.ftc.teamcode.Laser;
import org.firstinspires.ftc.teamcode.PIDCore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Config
@TeleOp
public class BabyDrive extends OpMode {

    public static Follower follower;

    public static double inPower = 0;

    public static double STARTING_X = 55.92558139534884;

    public static double STARTING_Y = 8.037209302325575;

    public static double STARTING_HEADING = 180;


    public static double LIMELIGHT_TARGET = 4;

    Laser laserSensor;

    ElapsedTime ballGrabTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static Pose defaultStartingPose = new Pose(STARTING_X, STARTING_Y, Math.toRadians(STARTING_HEADING));

    public static Pose startingPose;

    public static double DRIVE_SHOOT_REDUCER_COEFFICENT = 0.2;

    public static double AUTO_REDUCER = 0.75; //0.65

    public static DcMotorEx intake;

    public static double intakeReducer = 0.75;

    public static double driveReducer = 1;

    public static boolean isReduced = false;

    public static double RESET_HEADING_DEG = 180;

    public static double ALLOWED_HEADING_ERROR_DEG = 0.15;

    boolean wasMoved = false;

    public static double PEDRO_STANDING_HEADING_CONSTRAINT = Math.toDegrees(0.007);

    public static boolean automatedDrive = false;

    public static boolean useDefaultPose = false;

    public static double LL_KP = 0.008;


    public static int ballCount = 0;

    public static Pose targetPose;

    boolean threeBalls = false;

    public static Pose holdingPose;

    public Limelight3A limelight;


    public static double FAILSAFE_STICK_TRIGGER = 0.1;

    public static double X_TOLERANCE = 2;
    public static double Y_TOLERANCE = 2;
    public static double HEADING_TOLERANCE_DEG = 5;

    enum GAMEPAD_COLORS {
        RED,
        GREEN,
        BLUE
    }

    public static double luigiServoIntakeOffset = 0;

    public static double PATH_HEADING_OFFSET = 0;

    public static Supplier<PathChain> pathChain;

    public static double lastSign;

    public static boolean inFWD = false;

    OpShooterCore shooterCore = new OpShooterCore(telemetry);
    public static double intakeGamepad;

    public static boolean BLUE_ALLIANCE = true;

    public static boolean INTAKE_RUN_FWD = false;

    public static boolean INTAKE_RUN_REV = false;

    boolean LIMELIGHT_ALIGNED = false;

    public static boolean BOOT_RUN = false;

    public static ModeCore.ALLIANCE currAlliance;

    Kickstand kickstand;

    public static double INTAKE_THRESHOLD = 0.15;

    public static boolean IS_ROBOT_CENTRIC = true;

    boolean firstEntry = true;
    boolean currentlyStill = false;
    boolean previouslyStill = false;
    boolean currentlyMoved = false;
    boolean previouslyMoved = false;

    private static PathChain targetPath;

    PrismCore prismCore = new PrismCore();

    DrivetrainCore dtCore = new DrivetrainCore();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prismCore.Init(hardwareMap);
        ballCount = 0;
        kickstand = new Kickstand(hardwareMap, telemetry, gamepad1);
        laserSensor = new Laser(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorSimple.Direction inDir = inFWD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        currAlliance = BLUE_ALLIANCE ? ModeCore.ALLIANCE.BLUE : ModeCore.ALLIANCE.RED;
        intake.setDirection(inDir);
        ModeCore.currentDriveMode = ModeCore.DRIVE_MODE.MANUAL_DRIVE;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        follower = Constants.createFollower(hardwareMap);
        startingPose = useDefaultPose ? defaultStartingPose : PoseStorage.currentPose;
        follower.setStartingPose(startingPose);
        shooterCore.init(hardwareMap);
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.constant(targetPose.getHeading() + Math.toRadians(PATH_HEADING_OFFSET)))
                .build();
        dtCore.Init(hardwareMap);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        ballGrabTimer.reset();
        prismCore.LL_BAD();
        prismCore.BAR_LIGHT();
        shooterCore.setCRPower(-1);
    }

    public void updatePose(){
        follower.update();
        PoseStorage.currentPose = follower.getPose();
    }

    @Override
    public void stop(){
        prismCore.CLEAR();
    }

    @Override
    public void loop() {
        telemetry.addData("Current Mode: ", ModeCore.currentDriveMode);
        shooterCore.FlysPIDControl();
        updatePose();
        shooterCore.updateColors();
        telemetry.addData("Vel: ", shooterCore.fly.getVelocity());
        telemetry.addData("Ver: ", shooterCore.fry.getVelocity());
        telemetry.addData("Desired Vel: ", shooterCore.load_fly_expected_vel());
        telemetry.addData("Desired Ver: ", shooterCore.load_fry_expected_vel());
        telemetry.addData("Follower X: ", follower.getPose().getX());
        telemetry.addData("Follower Y: ", follower.getPose().getY());
        telemetry.addData("Follower Heading: ", follower.getPose().getHeading());
        telemetry.addData("Ball Count: ", ballCount);
        telemetry.update();
        follower.setTeleOpDrive(0, -gamepad1.left_stick_x * driveReducer, -gamepad1.right_stick_x * driveReducer, IS_ROBOT_CENTRIC);
        ModeCore.BabyModeCoreHandler(shooterCore);
        shooterCore.shooting_loop();
        handleShootingInputs();
        if (targetPose != null) {
            updatePathToFollow();
        }
        setGamepadLeds(GAMEPAD_COLORS.GREEN, GAMEPAD_COLORS.RED);
    }

    public void handleShootingInputs(){
        boolean flysAreRunning = shooterCore.fly.getVelocity() >= OpShooterCore.RUNNING_SPEEDS && shooterCore.fry.getVelocity() >= OpShooterCore.RUNNING_SPEEDS;
        if (gamepad2.dpadDownWasPressed() && flysAreRunning) {
            shooterCore.start_shoot_once();
            isReduced = true;
        }
        if (gamepad2.dpadUpWasPressed()){
            shooterCore.stop_shoot_once();
            isReduced = false;
        }
    }

    private void setGamepadLeds(GAMEPAD_COLORS oneColor, GAMEPAD_COLORS twoColor){
        if (oneColor == GAMEPAD_COLORS.RED) {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == GAMEPAD_COLORS.BLUE) {
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (oneColor == GAMEPAD_COLORS.GREEN) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.RED) {
            gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.BLUE) {
            gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (twoColor == GAMEPAD_COLORS.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updatePathToFollow(){
        targetPath = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }

}
