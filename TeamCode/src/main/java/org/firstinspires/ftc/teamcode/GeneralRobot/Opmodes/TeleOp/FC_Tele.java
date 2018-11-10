package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

@TeleOp(name="FC Teleop", group="Tele")
public class FC_Tele extends OpMode{
    Robot robotClass;
    public Servo flip;


    public void init(){
        robotClass = new Robot(hardwareMap, gamepad1, telemetry);
        flip = hardwareMap.servo.get("flip");
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        robotClass.lift.run();
        double bl = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double br = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double fl = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double fr = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double[] drivePowers = {fl, fr, bl, br};
        robotClass.mecanumDrive.drive(drivePowers);
        if(gamepad1.x)
            flip.setPosition(1);
    }
}
