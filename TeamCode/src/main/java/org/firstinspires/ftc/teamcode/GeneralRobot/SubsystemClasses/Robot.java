package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.OpenCVClass;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.Vision;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.VuforiaClass;

public class Robot
{
    //Creates an array of subsystems and vision things, and a class object for each subsystem
    public MecanumDrive mecanumDrive;
    public Subsystem[] subList = {mecanumDrive};
    public Vision vuforiaClass,opencvClass;
    public Vision[] visList = {vuforiaClass,opencvClass};

    //Sets up all of the needed objects, should also update the values in the list
    public Robot(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(hwmap,gamepad,telemetry);
        vuforiaClass = new VuforiaClass(hwmap,telemetry);
        opencvClass = new OpenCVClass(hwmap,telemetry);
    }

    //Initialize all of the subsystems
    public void initializeRobot() {
        for(int x=0;x<subList.length;x++){
            subList[x].initialize();
        }
    }
    //Initialize all of the vision classes
    public void initializeVision(){
        for(int x=0;x<visList.length;x++){
            visList[x].initialize();
        }
    }
    //brake all of the subsystems
    public void brake(){
        for(int x=0;x<subList.length;x++){
            subList[x].brake();
        }
    }

    //run all of the subsystems
    public void run(){
        for(int x=0;x<subList.length;x++){
            subList[x].run();
        }
    }
}
