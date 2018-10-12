package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

@TeleOp(name="FC Teleop", group="Tele")
public class FC_Tele extends OpMode{
    Robot robotClass;

    public void init(){
        robotClass = new Robot(hardwareMap, gamepad1, telemetry);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        robotClass.run();
    }
}
