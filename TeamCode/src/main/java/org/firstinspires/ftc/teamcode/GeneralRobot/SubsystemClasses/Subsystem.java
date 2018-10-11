package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {
    //Creates objects to be accessed by all the subsytems
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Gamepad gamepad;
    //provides values to these variables passed up
    public Subsystem(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        this.hardwareMap = hwmap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    //the three methods used by all the subsystems, can be used in the Robot class
    public abstract void run();
    public abstract void initialize();
    public abstract void brake();
}
