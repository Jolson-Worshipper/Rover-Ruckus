package org.firstinspires.ftc.teamcode.GeneralRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Subsystem;

@TeleOp(name="driveTest", group="Tele")
public class driveTest extends OpMode{
    Subsystem mechDrive;

    public void init(){
        mechDrive = new MecanumDrive(hardwareMap, gamepad1, telemetry);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        mechDrive.run();
    }
}
