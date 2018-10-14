package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    private CRServo intakeFront,intakeBack;
    private final double INTAKE_POWER = .3;
    public Intake(HardwareMap hwMap, Gamepad gamepad, Telemetry telem){
        super(hwMap,gamepad,telem);
        intakeFront = hardwareMap.crservo.get("intakeFront");
        intakeBack = hardwareMap.crservo.get("intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void initialize(){}
    public void run(){
        intakeBack.setPower(.5+(gamepad.left_trigger-gamepad.right_trigger)*INTAKE_POWER);
        intakeFront.setPower(.5+(gamepad.left_trigger-gamepad.right_trigger)*INTAKE_POWER);
    }
    public void brake(){
        intakeBack.setPower(.5);
        intakeFront.setPower(.5);
    }
}
