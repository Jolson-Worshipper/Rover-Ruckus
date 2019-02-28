package org.firstinspires.ftc.teamcode.GeneralRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Lift;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Subsystem;

@TeleOp(name="DC Test", group="Tele")
public class DisconnectTest extends OpMode{
    public void init(){ }
    public void loop() {
        new Thread() {
            public void run() {
                throw new RuntimeException("boi ur bad!"); }
        }.start();
    }
}