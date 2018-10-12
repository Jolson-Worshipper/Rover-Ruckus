package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem{
    //Declares motors for this class
    public DcMotor liftMotor;
    ElapsedTime timer;

    private final double WHEEL_RADIUS = 1/6;
    private final double maxAccel = 1; //Feet/sec^2
    private final double maxVelocity = .8; //Feet/sec

    public Lift(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        super(hwmap,gamepad,telemetry);

        //Creates new liftMotor on the phones/hardwaremap
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        useEncoders();

        timer = new ElapsedTime();
    }

    //Basic teleop method, run based on the powers of the triggers
    public void run(){
        liftMotor.setPower(gamepad.right_trigger-gamepad.left_trigger);
    }

    //if you call this, just brake the motors
    public void brake(){
        liftMotor.setPower(0);
    }

    //NOT FINISHED
    public void descend(int destination){
        liftMotor.setPower(.5);
    }

    public void initialize(){
        //Does nothing
    }

    //Motion profiling attempt that ideally works
    public void motionProfiling(double destinationDistance){
        //Sets the time needed to accelerate and distance
        double accelTime = maxVelocity/maxAccel;
        double accelDistance = (.5*maxAccel*accelTime*accelTime)*268.8*WHEEL_RADIUS/Math.PI; //Pretend this is converted to encoder value units
        double currentVelocity = 0;
        timer.reset();
        resetEncoders();
        //While it should be accelerating, do so at a predetermined rate
        while(currentVelocity<.8 && destinationDistance/2<liftMotor.getCurrentPosition()){
            currentVelocity = maxAccel*timer.seconds();
            liftMotor.setPower(1/maxVelocity*currentVelocity);
        }
        //While it's at max velocity, stay there until it should decellerate
        while(destinationDistance-accelDistance>liftMotor.getCurrentPosition()){
            liftMotor.setPower(1);
        }
        timer.reset();
        //Until it reaches its final distance, decellerate
        while(destinationDistance>liftMotor.getCurrentPosition()){
            currentVelocity = maxVelocity-maxAccel*timer.seconds();
            liftMotor.setPower(1/maxVelocity*currentVelocity);
        }
        //Stop in case it tries to go slightly negative
        brake();

    }

    public void useEncoders() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int getEncoders(){
        return liftMotor.getCurrentPosition();
    }
}