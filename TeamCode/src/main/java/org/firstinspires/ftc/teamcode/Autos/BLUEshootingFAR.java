package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Temporary.ModeCore;
import org.firstinspires.ftc.teamcode.Temporary.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;


@Config
@Autonomous(name = "BLUE Shooting FAR", group = "BLUE")
@Configurable // Panels
public class BLUEshootingFAR extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static long myEntryTime = 0;

    public static int PATH_INDEX = 1;
    Telemetry dashTele = dashboard.getTelemetry();

    public static int INTAKE_POWER_OFFSET = 100;

    public static long TIMEOUT = 3000;

    public static int HEADING_OFFSET = 8;

    public static boolean HOLD_END = true;

    public static double PICKUP_POWER = 0.45;

    public static double ROLLBACK_POWER = 0.75;

    public boolean firstTimeCR = true;

    public boolean secondTimeCR = true;
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();
  public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
  private int pathState; // Current autonomous path state (state machine)

    public static int L_VEL = 1000;

    public static int R_VEL = 1000;
    private final Pose startPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose shootFar1 = new Pose(62, 11.5, Math.toRadians(100)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collectBalls1 = new Pose(19.5, 33, Math.toRadians(0));
    private final Pose collectBalls1ControlPoint1 = new Pose(69.62385321100919, 31.29701834862384);
    private final Pose shootFar2 = new Pose(62, 11.5, Math.toRadians(100));
    private final Pose parkingPose = new Pose(38.75, 33.5, Math.toRadians(90));

    private PathChain firstPath, collect1Path, thirdPath, parkingPath;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
  public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    shooterAutoCore.init(hardwareMap);
    shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow);

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);

    buildPaths();

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  public void buildPaths(){
        PathCallback FirstShoot = new PathCallback() {
            @Override
            public boolean run() {
                follower.pausePathFollowing();
                while (!shooterAutoCore.shoot(3, dashTele)){
                    dashTele.update();
                }
                shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
                shooterAutoCore.in();
                follower.setMaxPower(PICKUP_POWER);
                shooterAutoCore.spinUpFlys(L_VEL, R_VEL );
                follower.resumePathFollowing();
                return true;
            }

            @Override
            public void initialize() {
                if (firstTimeCR) {
                    ElapsedTime hey = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                    hey.reset();
                    follower.pausePathFollowing();
                    while (hey.time(TimeUnit.MILLISECONDS) < TIMEOUT){
                        dashTele.addData("Timer: ", hey.now(TimeUnit.MILLISECONDS));
                        follower.update();
                        dashTele.update();
                    }
                    follower.resumePathFollowing();
                    dashTele.addData("Timer: ", hey.now(TimeUnit.MILLISECONDS));
                    follower.update();
                    dashTele.update();
                    ShooterAutoCore.failsafeTimer.reset();
                    shooterAutoCore.setCRPower(1, dashTele);
                    shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
                    firstTimeCR = false;
                }
            }

            @Override
            public boolean isReady() {
                return true;
            }

            @Override
            public int getPathIndex() {
                return 0;
            }
        };

      PathCallback SecondShoot = new PathCallback() {
          @Override
          public boolean run() {
              follower.pausePathFollowing();
              while (!shooterAutoCore.intakeShoot(3, dashTele)){
                  shooterAutoCore.luigiServo.setPosition(ModeCore.BLUE_INTAKE_LEFT_FAR_SERVO);
                  dashTele.update();
              }
              shooterAutoCore.setCRPower(0, dashTele);
              shooterAutoCore.spinUpFlys(0, 0);
              follower.resumePathFollowing();
              return true;
          }

          @Override
          public void initialize() {
              if (secondTimeCR) {
                  follower.resumePathFollowing();
                  ShooterAutoCore.failsafeTimer.reset();
                  shooterAutoCore.luigiServo.setPosition(ModeCore.BLUE_INTAKE_LEFT_FAR_SERVO);
                  shooterAutoCore.in();
                  shooterAutoCore.setCRPower(1, dashTele);
                  secondTimeCR = false;
              }
          }

          @Override
          public boolean isReady() {
              return true;
          }

          @Override
          public int getPathIndex() {
              return 0;
          }
      };

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootFar1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootFar1.getHeading())
                .build();

        collect1Path = follower.pathBuilder()
                .addPath(new BezierCurve(shootFar1, collectBalls1ControlPoint1, collectBalls1))
                .setConstantHeadingInterpolation(collectBalls1.getHeading())
                .addCallback(FirstShoot)
                .build();

        thirdPath = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls1, shootFar2))
                .setLinearHeadingInterpolation(collectBalls1.getHeading(), shootFar2.getHeading())
                .build();

        parkingPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFar2, parkingPose))
                .setLinearHeadingInterpolation(shootFar2.getHeading(), parkingPose.getHeading())
                .addCallback(SecondShoot)
                .build();

  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    autonomousPathUpdate(); // Update autonomous state machine
    //shooterAutoCore.power_surge(150);

    // Log values to Panels and Driver Station
      dashTele.update();
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
  }

  public void autonomousPathUpdate() {
    switch (pathState){
        case 0:
            //shooterAutoCore.in();
            shooterAutoCore.spinUpFlys(L_VEL, R_VEL);
            shooterAutoCore.setLauncherPos(ModeCore.BLUE_HOPPER_FAR_LEFT_LAUNCHER);
            follower.followPath(firstPath);
            setPathState(1);
            break;
        case 1:
            if (!follower.isBusy()){
                follower.followPath(collect1Path);
                setPathState(2);
                break;
            }
        case 2:
            if (!follower.isBusy()) {
                follower.setMaxPower(ROLLBACK_POWER);
                follower.followPath(thirdPath);
                dashTele.update();
                setPathState(3);
                break;
            }
        case 3:
            if (!follower.isBusy()){
                follower.followPath(parkingPath);
                setPathState(4);
                break;
            }
        case 4:
            if (!follower.isBusy()) {
                PoseStorage.currentPose = follower.getPose();
                shooterAutoCore.spinUpFlys(0, 0);
                dashTele.update();
                setPathState(-1);
                break;
            }
    }
  }
}
    