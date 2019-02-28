package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem{
    //Declares motors for this class
    public DcMotor liftMotor;

    public Lift(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        super(hwmap,gamepad,telemetry);
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Basic teleop method, run based on the powers of the triggers
    public void run(){
        liftMotor.setPower(gamepad.right_trigger-gamepad.left_trigger);
    }

    //if you call this, just brake the motors
    public void brake(){
        liftMotor.setPower(0);
    }

    public void initialize(){
    }
    public void drive(double power){
        liftMotor.setPower(power);
    }

    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}