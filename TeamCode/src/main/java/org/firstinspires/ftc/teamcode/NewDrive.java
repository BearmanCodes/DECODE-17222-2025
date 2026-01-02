package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class NewDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Init();
        waitForStart();
        while (!isStopRequested()){

        }
    }

    public void Init(){
        //SetUp CrServos, Left (Reversed) and Right
        //SetUp flywheels, Left (Reversed) and Right
        //SetUp Intake spinny, Reversed
        /*SetUp Linear Actuators left and right, laR.setPwmRange(new PwmControl.PwmRange(1000, 2000));
                //fully retract 0, fully extend 1
                laR.setPwmEnable();*/
        //Odometry:
            //hardwareMap
            //limelight.pipelineSwitch(0);7
            //offsets: 7.025, 0.95, inch
            //setEncoderResolutions(4_Bar_Pod)
            //setEncoderDirections(Reversed, Reversed)
            //odo.resetPosAndImu();
            //limelight.start();
    }
}
