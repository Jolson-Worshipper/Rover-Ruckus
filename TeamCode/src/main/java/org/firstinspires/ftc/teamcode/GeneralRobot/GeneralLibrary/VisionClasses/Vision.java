package org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Vision {
    //Creates instances of the hardwaremap and telemetry to be used in the other vision classes
    protected HardwareMap hardwaremap;
    protected Telemetry telemetry;

    public Vision(HardwareMap hardwaremap,Telemetry telemetry){
        this.hardwaremap = hardwaremap;
        this.telemetry = telemetry;
    }

    //Most important functions are to initialize the vision, run the code, and close the vision
    public abstract void initialize();
    public abstract void run();
    public abstract void close();
}
