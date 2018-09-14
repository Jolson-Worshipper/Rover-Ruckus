package org.firstinspires.ftc.teamcode.GeneralRobot.subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by Brown on 4/24/2018.
 */

public class TankDrive {

    private HardwareMap hardwareMap;
    public Telemetry telemetry;

    private Gamepad gamepad;
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public BNO055IMU gyro;

    public double heading;
    public double DRIVE_POWER = .4;
    public double leftPower,rightPower;
    private static final double WHEEL_RADIUS = 1/6;
    double maxAccel = 1; //Feet/sec^2
    double maxVelocity = .8; //Feet/sec

    ElapsedTime timer;

    public enum encoderMode{
        runToPosition,runUsingEncoders,runWIthoutEncoders
    }
    public encoderMode tankEncoderMode;


    public TankDrive(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        this.hardwareMap = hwmap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        InitializeGyro();
        timer.reset();
    }


    public double[] getThreshHold(double[] powers){
        double maxPower = 0;
        for(double power : powers){
            if(Math.abs(power)>maxPower)
                maxPower = power;
        }
        for(int i=0;i<powers.length;i++){
            powers[i] = powers[i]*DRIVE_POWER/Math.abs(maxPower);
        }
        return powers;
    }

    public void drive(double[] powers) {
        double[] drivePowers = getThreshHold(powers);
        frontLeft.setPower(drivePowers[0]);
        frontRight.setPower(drivePowers[1]);
        backLeft.setPower(drivePowers[0]);
        backRight.setPower(drivePowers[1]);
    }

    public void InitializeGyro() {
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);
    }

    public void updateGyro() {
        heading = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

        if (heading > 0)
            heading = heading;
        else
            heading = heading + 360;
        telemetry.addData("Gyro Heading",heading);
    }


    public void genDrive() {
        double[] drivePowers = {gamepad.left_stick_y,gamepad.right_stick_y};
        drive(drivePowers);
    }

    public void arcadeDrive(){
        leftPower = gamepad.left_stick_y+gamepad.left_stick_x;
        rightPower = gamepad.left_stick_y-gamepad.left_stick_x;
        double[] drivePowers = {leftPower,rightPower};
        drive(drivePowers);
    }

    public void brake() {
        double[] drivePowers = {0,0};
        drive(drivePowers);
    }

    public void restistOnBrake(){
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void floatOnBrake(){
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void useEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tankEncoderMode = encoderMode.runUsingEncoders;
    }

    public void useNoEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tankEncoderMode = encoderMode.runWIthoutEncoders;
    }

    public void setDriveEncoders(double powerfl, double powerfr, double powerbl, double powerbr, int fl, int fr, int bl, int br) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tankEncoderMode = encoderMode.runToPosition;

        frontLeft.setTargetPosition(fl + frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(fr + frontRight.getCurrentPosition());
        backLeft.setTargetPosition(bl + backLeft.getCurrentPosition());
        backRight.setTargetPosition(br + backRight.getCurrentPosition());

        frontLeft.setPower(powerfl);
        frontRight.setPower(powerfr);
        backLeft.setPower(powerbl);
        backRight.setPower(powerbr);
    }

    public void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void motionProfiling(double destinationDistance){
        double accelTime = maxVelocity/maxAccel;
        double accelDistance = (.5*maxAccel*accelTime*accelTime)*268.8*WHEEL_RADIUS/Math.PI; //Pretend this is converted to encoder value units
        double currentVelocity = 0;
        double startingTime = timer.seconds();
        resetEncoders();

        while(currentVelocity<.8 && destinationDistance/2<(frontLeft.getCurrentPosition()+frontRight.getCurrentPosition())/2){
            currentVelocity = maxAccel*(timer.seconds()-startingTime);
            double[] drivePowers = {1/maxVelocity*currentVelocity,1/maxVelocity*currentVelocity};
            drive(drivePowers);
        }

        while(destinationDistance-accelDistance>(frontLeft.getCurrentPosition()+frontRight.getCurrentPosition())/2){
             double[] drivePowers = {1,1};
            drive(drivePowers);
        }
        startingTime = System.currentTimeMillis()/1000;

        while(destinationDistance>(frontLeft.getCurrentPosition()+frontRight.getCurrentPosition())/2){
            currentVelocity = maxVelocity-maxAccel*(timer.seconds()/1000-startingTime);
            double[] drivePowers = {1/maxVelocity*currentVelocity,1/maxVelocity*currentVelocity};
            drive(drivePowers);
        }
        brake();

    }
}