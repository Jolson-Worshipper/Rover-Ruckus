package org.firstinspires.ftc.teamcode.GeneralRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Lift;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Subsystem;

@TeleOp(name="liftTest", group="Tele")
public class liftTest extends OpMode{
    Lift lift;
    MecanumDrive drive;
    public DcMotor liftMotor,liftMotor2;


    public void init(){
        lift = new Lift(hardwareMap, gamepad1, telemetry);
        drive = new MecanumDrive(hardwareMap,gamepad1,telemetry);
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}

    public void init_loop(){}

    public void start(){
    }

    public void loop(){
        /*lift.run();
        drive.run();*/
        liftMotor.setPower(gamepad1.right_trigger);
        liftMotor2.setPower(gamepad1.left_trigger);
    }
}

