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
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;


@Config
@Autonomous(name = "Shooting Autonomous", group = "Autonomous")
@Configurable // Panels
public class ShootingAutonomous extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int PATH_INDEX = 1;
    Telemetry dashTele = dashboard.getTelemetry();

    public static int INTAKE_POWER_OFFSET = 100;

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

    public static int L_VEL = 900;

    public static int R_VEL = 900;
    private final Pose startPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose endPose1 = new Pose(56.515188335358445, 88, Math.toRadians(135)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose endPose2 = new Pose(56.515188335358445, 88, Math.toRadians(135 + HEADING_OFFSET));

    //private final Pose homePose = new Pose(56.13001215066828, 55.822600243013355, Math.toRadians(90));

    private final Pose homePose = new Pose(19, 53, Math.toRadians(0));

    private final Pose homePoseCtrlPoint = new Pose(52.880315917375455, 53.464155528554066);

    private final Pose collectBalls1 = new Pose(19, 78.25, Math.toRadians(0));

    private final Pose collectBalls1ControlPoint = new Pose(64.69205408208279, 78.5613608748481);

    private PathChain firstPath, endingPath, secondBarragePath, homePath;

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
                shooterAutoCore.spinUpFlys(L_VEL - INTAKE_POWER_OFFSET, R_VEL - INTAKE_POWER_OFFSET);
                follower.resumePathFollowing();
                return true;
            }

            @Override
            public void initialize() {
                if (firstTimeCR) {
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
                  dashTele.update();
              }
              follower.setMaxPower(PICKUP_POWER);
              shooterAutoCore.in();
              shooterAutoCore.setCRPower(0, dashTele);
              shooterAutoCore.spinUpFlys(0, 0);
              follower.resumePathFollowing();
              return true;
          }

          @Override
          public void initialize() {
              if (secondTimeCR) {
                  shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
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
              return PATH_INDEX;
          }
      };

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose1.getHeading())
                .build();

        endingPath = follower.pathBuilder()
                .addPath(new BezierCurve(endPose1, collectBalls1ControlPoint, collectBalls1))
                .setConstantHeadingInterpolation(collectBalls1.getHeading())
                .addCallback(FirstShoot)
                .build();

        secondBarragePath = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls1, endPose2))
                .setLinearHeadingInterpolation(collectBalls1.getHeading(), endPose2.getHeading())
                .addPath(new BezierCurve(endPose2, homePoseCtrlPoint, homePose))
                //.addPath(new BezierLine(endPose2, homePose))
                //.setLinearHeadingInterpolation(endPose2.getHeading(), homePose.getHeading())
                .setConstantHeadingInterpolation(homePose.getHeading())
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
            shooterAutoCore.setLauncherPos(ShooterAutoCore.laInitPos);
            follower.followPath(firstPath);
            setPathState(1);
            break;
        case 1:
            if (!follower.isBusy()){
                follower.followPath(endingPath);
                setPathState(2);
                break;
            }
        case 2:
            if (!follower.isBusy()) {
                follower.setMaxPower(ROLLBACK_POWER);
                follower.followPath(secondBarragePath);
                dashTele.update();
                setPathState(3);
                break;
            }
        case 3:
            if (!follower.isBusy()){
                shooterAutoCore.spinUpFlys(0, 0);
                dashTele.update();
                setPathState(-1);
                break;
            }
    }
  }


}
    