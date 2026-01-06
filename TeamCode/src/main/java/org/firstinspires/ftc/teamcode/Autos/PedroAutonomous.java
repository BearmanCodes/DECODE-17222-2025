package org.firstinspires.ftc.teamcode.Autos;
    import com.acmerobotics.dashboard.FtcDashboard;
    import com.pedropathing.util.Timer;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.bylazar.configurables.annotations.Configurable;
    import com.bylazar.telemetry.TelemetryManager;
    import com.bylazar.telemetry.PanelsTelemetry;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import com.pedropathing.geometry.BezierCurve;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.paths.PathChain;
    import com.pedropathing.geometry.Pose;
    
    
    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class PedroAutonomous extends OpMode {
      private TelemetryManager panelsTelemetry; // Panels Telemetry instance

        private FtcDashboard dashboard = FtcDashboard.getInstance();

        private Telemetry dashTele = dashboard.getTelemetry();
        public ShooterAutoCore shooterAutoCore = new ShooterAutoCore();

        public static int L_VEL = 1100;

        public static int R_VEL = 1100;
      public Follower follower; // Pedro Pathing follower instance
        Timer pathTimer;
        Timer opmodeTimer;
      private int pathState; // Current autonomous path state (state machine)
        private final Pose startPose = new Pose(40.465, 135.740, Math.toRadians(0)); // Start Pose of our robot.
        private final Pose controlPoint1 = new Pose(58.130, 77.395, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose endPose1 = new Pose(18.82790697674418, 84.3906976744186, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

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

        shooterAutoCore.init(hardwareMap);
        shooterAutoCore.spinUpFlys(L_VEL, R_VEL);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        firstPath = follower.pathBuilder().addPath(new BezierCurve(startPose, controlPoint1, endPose1)).setLinearHeadingInterpolation(0, 0).build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
      }
      
      @Override
      public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
          dashTele.addData("Pose X: ", follower.getPose().getX());
          dashTele.addData("Pose Y: ", follower.getPose().getY());
          dashTele.addData("Pose Heading: ", follower.getPose().getHeading());
          dashTele.update();
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
            dashTele.addData("Pose X: ", follower.getPose().getX());
            dashTele.addData("Pose Y: ", follower.getPose().getY());
            dashTele.addData("Pose Heading: ", follower.getPose().getHeading());
            dashTele.update();
            shooterAutoCore.luigiServo.setPosition(ShooterAutoCore.luigiBlock);
            shooterAutoCore.setCRPower(1, dashTele);
            setPathState(0);

      }
  

      public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                shooterAutoCore.shoot(3, dashTele);
                dashTele.addData("Pose X: ", follower.getPose().getX());
                dashTele.addData("Pose Y: ", follower.getPose().getY());
                dashTele.addData("Pose Heading: ", follower.getPose().getHeading());
                dashTele.update();
            case 1:
                if (!follower.isBusy()){
                    setPathState(-1);
                    shooterAutoCore.stop();
                    break;
                }
        }
      }
      
      
    }
    