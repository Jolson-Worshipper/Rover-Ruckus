package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.Vision;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.VuforiaClass;

public class Robot
{
    //Creates an array of subsystems and vision things, and a class object for each subsystem
    public MecanumDrive mecanumDrive;
    /*public Bucket bucket;
    public Extender extender;
    public Intake intake;*/
    public Lift lift;
    public Subsystem[] subList = {mecanumDrive/*,bucket,extender,intake*/,lift};
    public VuforiaClass vuforiaClass;
    public Vision[] visList = {vuforiaClass};

    //Sets up all of the needed objects, should also update the values in the list
    public Robot(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(hwmap,gamepad,telemetry);
        /*bucket = new Bucket(hwmap,gamepad,telemetry);
        extender = new Extender(hwmap,gamepad,telemetry);
        intake = new Intake(hwmap,gamepad,telemetry);*/
        lift = new Lift(hwmap,gamepad,telemetry);
        vuforiaClass = new VuforiaClass(hwmap,telemetry);
    }
    //Initialize all of the subsystems
    /*public void initializeRobot() {
        for(Subsystem sub : subList){
            sub.initialize();
        }
    }*/
    //Initialize all of the vision classes
    public void initializeVision(){
        for(Vision vis : visList){
            vis.initialize();
        }
    }
    public void closeVision(){
        for(Vision vis : visList){
            vis.close();
        }
    }
    //brake all of the subsystems
    public void brake(){
        for(Subsystem sub : subList){
            sub.brake();
        }
    }

    //run all of the subsystems
    public void run(){
        for(Subsystem sub : subList){
            sub.run();
        }
    }
}
