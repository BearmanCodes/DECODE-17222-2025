package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Shooting Autonomous", group = "Autonomous")
@Configurable // Panels
public class ShootingAutonomous extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public IntakeAutoCore intakeAutoCore = new IntakeAutoCore();

    public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();
  public Follower follower; // Pedro Pathing follower instance
    Timer pathTimer;
    Timer opmodeTimer;
  private int pathState; // Current autonomous path state (state machine)
    private final Pose startPose = new Pose(55.92558139534884, 8.037209302325575, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose endPose1 = new Pose(47.460465116279074, 95.60930232558137, Math.toRadians(135)); // Highest (First Set) of Artifacts from the Spike Mark.

    private PathChain firstPath;

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

    intakeAutoCore.init(hardwareMap);
    shooterAutoCore.init(hardwareMap);

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);

    firstPath = follower.pathBuilder().addPath(new BezierLine(startPose, endPose1)).setLinearHeadingInterpolation(startPose.getHeading(), endPose1.getHeading()).build();

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    autonomousPathUpdate(); // Update autonomous state machine
    //shooterAutoCore.power_surge(150);

    // Log values to Panels and Driver Station
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
            shooterAutoCore.spinUpFlys(1100, 1100);
            shooterAutoCore.setLauncherPos(0.55);
            follower.followPath(firstPath);
            setPathState(1);
            break;
        case 1:
            if (!follower.isBusy()){
                shooterAutoCore.setCRPower(1);
                setPathState(-1);
                break;
            }
    }
  }


}
    