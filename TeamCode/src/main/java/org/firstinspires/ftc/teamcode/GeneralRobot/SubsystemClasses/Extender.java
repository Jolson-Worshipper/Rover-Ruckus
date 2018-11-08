package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.GamepadClasses.GamepadClass;

public class Extender extends Subsystem {
    private DcMotor extend;
    private GamepadClass gamepadClass;
    private final double EXTEND_POWER = .7;

    public Extender(HardwareMap hwMap, Gamepad gamepad, Telemetry telem){
        super(hwMap,gamepad,telem);
        gamepadClass = new GamepadClass(gamepad);
        extend = hardwareMap.dcMotor.get("extender");
    }
    public void initialize(){}
    public void run(){
        gamepadClass.update();
        if(gamepadClass.A.getVal())
            extend.setPower(EXTEND_POWER);
        else if(gamepadClass.B.getVal())
            extend.setPower(-EXTEND_POWER);
        else
            extend.setPower(0);
    }
    public void brake(){
        extend.setPower(0);
    }
}