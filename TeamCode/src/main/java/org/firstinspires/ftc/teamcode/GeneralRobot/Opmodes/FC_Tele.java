package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

@TeleOp(name="FC Teleop", group="Tele")
public class FC_Tele extends OpMode{
    Robot robotClass;

    @Override
    public void init(){
        robotClass = new Robot(hardwareMap, gamepad1, telemetry);
    }

    @Override
    public void loop(){
        robotClass.run();
    }
}
