package org.firstinspires.ftc.teamcode.GeneralRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Lift;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Subsystem;

@TeleOp(name="liftTest", group="Tele")
public class liftTest extends OpMode{
    Subsystem lift,drive;

    public void init(){
        lift = new Lift(hardwareMap, gamepad1, telemetry);
        drive = new MecanumDrive(hardwareMap,gamepad1,telemetry);
}

    public void init_loop(){}

    public void start(){}

    public void loop(){
        lift.run();
        drive.run();
    }
}

