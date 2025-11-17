package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class FileTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        InitFile();
        waitForStart();
        AppendData("The snow is yellow");
    }

    public void InitFile(){
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
        File directory = new File(directoryPath);
        directory.mkdir();
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(directoryPath + "/" + "yourmom.csv"))) {
            bw.write("X,Y,Z,Pitch,Roll,Yaw,Range,Bearing,Elevation,L vel,R vel,LA Pos");
            System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void AppendData(String output){
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(directoryPath + "/" + "yourmom.csv"))) {
            bw.newLine();
            bw.write(output);
            telemetry.log().add(output);
            telemetry.addLine("Successfully wrote to the file.");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("Error writing file.");
            telemetry.update();
        }
    }

}
