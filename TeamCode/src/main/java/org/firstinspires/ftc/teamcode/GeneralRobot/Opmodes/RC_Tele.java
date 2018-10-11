package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

@TeleOp(name="RC Teleop", group="Tele")
public class RC_Tele extends OpMode{
    Robot robotClass;

    @Override
    public void init(){
        robotClass = new Robot(hardwareMap, gamepad1, telemetry);
    }

    @Override
    public void loop(){
        robotClass.mecanumDrive.RCDrive();
        //robotClass.liftClass.run();
    }

}
