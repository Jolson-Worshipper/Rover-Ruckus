package org.firstinspires.ftc.teamcode.GeneralRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Extender;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Subsystem;

@TeleOp(name="extendTest", group="Tele")
public class extendTest extends OpMode{
    Subsystem extender;

    public void init(){
        extender = new Extender(hardwareMap, gamepad1, telemetry);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        extender.run();
    }
}
