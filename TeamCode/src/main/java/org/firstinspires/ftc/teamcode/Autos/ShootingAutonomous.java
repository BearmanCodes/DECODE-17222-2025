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


@Config
@Autonomous(name = "Shooting Autonomous", group = "Autonomous")
@Configurable // Panels
public class ShootingAutonomous extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTele = dashboard.getTelemetry();

    public boolean firstTimeCR = true;
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();
  public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
  private int pathState; // Current autonomous path state (state machine)

    public static int L_VEL = 900;

    public static int R_VEL = 900;
    private final Pose startPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose endPose1 = new Pose(56.515188335358445, 85.910085054678, Math.toRadians(135)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose endPose2 = new Pose(57.914945321992704, 16.797083839611176, Math.toRadians(180));
    private PathChain firstPath, endingPath;

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

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);

    buildPaths();

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  public void buildPaths(){
        PathCallback TestCallback = new PathCallback() {
            @Override
            public boolean run() {
                follower.pausePathFollowing();
                while (!shooterAutoCore.shoot(3, dashTele)){
                    dashTele.update();
                }
                follower.resumePathFollowing();
                return true;
            }

            @Override
            public void initialize() {
                if (firstTimeCR) {
                    shooterAutoCore.setCRPower(1, dashTele);
                    shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiFlow + ShooterAutoCore.KICK_ITERATOR);
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

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose1.getHeading())
                .build();

        endingPath = follower.pathBuilder()
                .addPath(new BezierLine(endPose1, endPose2))
                .setLinearHeadingInterpolation(endPose1.getHeading(), endPose2.getHeading())
                .addCallback(TestCallback)
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
                dashTele.update();
                shooterAutoCore.spinUpFlys(0, 0);
                setPathState(-1);
                break;
            }
    }
  }


}
    