package org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.GamepadClasses.GamepadClass;

public class MecanumDrive extends Subsystem{
    //Creates gamepad to use debouncing
    GamepadClass gamepadClass;

    //Creates all the hardware
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public BNO055IMU gyro;

    //Sets up all the drivivng variables (versions of the joysticks, and field centric variables
    private double jp,jTheta,theta,fr,fl,br,bl;
    private double leftY,leftX,rightX;
    public double heading;
    public final double DRIVE_POWER = 1;
    public double multiplier = 2;
    public double angleFromDriver = 0;

    public MecanumDrive(HardwareMap hwmap, Gamepad gamepad, Telemetry telemetry) {
        super(hwmap,gamepad,telemetry);
        //Sets up the new gamepad and all of the motors
        gamepadClass = new GamepadClass(gamepad);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        //Reverses 2 motors so it drives normally
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        InitializeGyro();
    }

    //function that takes the highest drive power and if it's over 1, tunes all of the powers to be <=1 and still proportional
    public double[] getThreshHold(double[] powers){
        //finds the largest drivePower
        double maxPower = 0;
        for(double power : powers){
            if(Math.abs(power)>maxPower)
                maxPower = power;
        }
        //If it's greater than the drivepower, tune them all to be in check
        if(maxPower>DRIVE_POWER){
            for (int i = 0; i < powers.length; i++) {
                powers[i] = powers[i] * DRIVE_POWER / Math.abs((maxPower));
            }
        }
        return powers;
    }

    //Drives all 4 motors at the set powers
    public void drive(double[] powers) {
        double[] drivePowers = getThreshHold(powers);
        frontLeft.setPower(drivePowers[0]);
        frontRight.setPower(drivePowers[1]);
        backLeft.setPower(drivePowers[2]);
        backRight.setPower(drivePowers[3]);
    }

    //Initializes the gyro with the values that we want
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

    //
    public void updateGyro() {
        //sets the heading to the current angle, if it's less than 0 compensate
        heading = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (heading < 0)
            heading += 360;
        telemetry.addData("Gyro Heading",heading);
    }

    //Runs the teleOp code, first  updating everything
    public void run() {
        gamepadClass.update();
        updateGyro();
        //If Y is pressed, set the current forward direction to wherever the robot is facing
        if (gamepadClass.Y.getVal())
            angleFromDriver += heading;
        //alters the joystick values ex. squaring, cubing, etc.
        alterJoystickValues();
        //converts joystick values to Field Centric
        setFieldCentricValues();
        //Drives at said powers
        drive(setFieldCentricDrivePowers());
    }

    public void RCDrive() {
        //alters the joystick values ex. squaring, cubing, etc.
        alterJoystickValues();
        //Drive at derived drive powers
        drive(setRobotCentricDrivePowers());
    }

    //Switch encoder  modes to using encoders
    public void useEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Set encoder mode to not using encoders
    public void useNoEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Set the encoder distance and speed for auto
    public void setDriveEncoders(double powerfl, double powerfr, double powerbl, double powerbr, int fl, int fr, int bl, int br) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(fl + frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(fr + frontRight.getCurrentPosition());
        backLeft.setTargetPosition(bl + backLeft.getCurrentPosition());
        backRight.setTargetPosition(br + backRight.getCurrentPosition());

        frontLeft.setPower(powerfl);
        frontRight.setPower(powerfr);
        backLeft.setPower(powerbl);
        backRight.setPower(powerbr);
    }

    //Reset encoders
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

    //Brake all motors
    public void brake() {
        double[] drivePowers = {0,0,0,0};
        drive(drivePowers);
    }

    //resist movement on brake
    public void restistOnBrake(){
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Move freely on brake
    public void floatOnBrake(){
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Initialize drivetrain
    public void initialize(){}

    //Squares/operates to affect the joystick value, if it's an even power then make sure to get the right sign
    public void alterJoystickValues(){
        if(multiplier%2 != 1 || multiplier!=1){
            rightX = Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, multiplier);
            leftX = Math.signum(gamepad.left_stick_x) * Math.pow(gamepad.left_stick_x, multiplier);
            leftY = Math.signum(gamepad.left_stick_y) * Math.pow(gamepad.left_stick_y, multiplier);
        }
        else{
            rightX = Math.pow(gamepad.right_stick_x, multiplier);
            leftX = Math.pow(gamepad.left_stick_x, multiplier);
            leftY = Math.pow(gamepad.left_stick_y, multiplier);
        }
    }

    //sets the field centric values of the joystick based on gamepad values
    public void setFieldCentricValues(){
        jTheta = Math.toDegrees(Math.atan2(-leftY,leftX));
        jp = Math.sqrt(leftX * leftX + leftY * leftY);
        if (jp > 1)
            jp = 1;
        theta = Math.toRadians(jTheta + angleFromDriver - heading);
    }

    //returns list of drivepowers based on field centric data from setFieldCentricValues
    public double[] setFieldCentricDrivePowers(){
        fl = (Math.sin(theta) + Math.cos(theta)) * jp / 2 + rightX;
        fr = (Math.sin(theta) - Math.cos(theta)) * jp / 2 - rightX;
        bl = (Math.sin(theta) - Math.cos(theta)) * jp / 2 + rightX;
        br = (Math.sin(theta) + Math.cos(theta)) * jp / 2 - rightX;
        double[] drivePowers = {fl, fr, bl, br};
        return drivePowers;
    }

    //returns list of drivepowers based on robot centric values
    public double[] setRobotCentricDrivePowers(){
        double bl = -leftY - leftX + rightX;
        double br = -leftY + leftX - rightX;
        double fl = -leftY + leftX + rightX;
        double fr = -leftY - leftX - rightX;
        double[] drivePowers = {fl, fr, bl, br};
        return drivePowers;
    }
}