package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.PIDClass;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses.OpenCVClass;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.AutoTransitioner;

@Autonomous(name="Auto Test (Depot)",group="Auto")
public class AutoMethods extends LinearOpMode {
    protected enum MineralLocation {
        LEFT,
        CENTER,
        RIGHT
    }
    public enum StartLocation {
        CRATER,
        DEPOT,
    }
    public Servo flip;
    protected ElapsedTime timer;
    protected Robot robot;
    private PIDClass turnPID;
    private OpenCVClass opencv;
    //Use this once we get 2 autos
    protected StartLocation startLocation = StartLocation.DEPOT;
    //Change this to RIGHT,LEFT,CENTER, to test the 3 mineral locations
    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    protected final double KP = .01;
    protected final double KI = .0025;
    protected final double KD = 0;
    //Tolerance that PID uses before accepting its angle
    protected final double TURN_TOLERANCE = 1;
    //Speeds of the robot
    protected final double AUTO_DRIVE_POWER = .5;
    protected final double PID_SPEED = .4;
    protected final double TURN_ANGLE = 40;
    //Distance to go forwards to leave the start
    protected final int MINERAL_ENCODER_SETUP = 700;
    //Distance to strafe sideways to get to mineral
    protected final int MINERAL_ENCODER_LEFT = 850;
    protected final int MINERAL_ENCODER_RIGHT = 900;
    //Distance to go forwards to hit mineral
    protected final int ENCODER_HIT_MINERAL = 975;
    //Distance to strafe sideways to get to depot
    protected final int MARKER_RIGHT = 1475;
    protected final int MARKER_LEFT = 1200;
    protected final int MARKER_CENTER = 1300;
    //Distance to go to square up
    protected final int PLACE_LEFT = 800;
    protected final int PLACE_CENTER = 600;
    protected final int PLACE_RIGHT = 400;
    //Park distance
    protected final int PARK = 30;
    //Time to use encoders at the start
    protected final double ENCODER_TIME = 2;
    //Time to rotate
    protected final double PID_TIME = 5;
    //Time to descend
    protected final long DESCEND_TIME = 15000;
    //Time to park/get over there
    protected final double PARK_TIME = 2;
    //Time to bring a servo down
    protected final long SERVO_TIME = 300;
    //TIme to strafe sideways to deposit marker
    protected final double STRAFE_TIME = 5;
    //Time to square up
    protected final double SQUARE_TIME = 3;

    public void runOpMode() throws InterruptedException{
        initOnce();
        //Inits team marker+passive hang
        while(opModeIsActive()&&!isStarted())
            initLoop();
        waitForStart();
        opencv.close();
        //descend();
        //brings the robot out of the park zone
        orientRobot();
        //strafes to then hits the mineral
        hitMineralDepot();
        //rotates, strafes sideways, deposits team marker
        markerOrientDepot(0);
        SquareUp();
        markerStrafe();
        //drives back and parks
        park(0);
        requestOpModeStop();
    }
    
    protected void initOnce(){
        flip = hardwareMap.servo.get("flip");
        timer = new ElapsedTime();
        opencv = new OpenCVClass(hardwareMap,telemetry);
        robot = new Robot(hardwareMap, gamepad1, telemetry);
        robot.mecanumDrive.restistOnBrake();
        turnPID = new PIDClass(KP,KI,KD);
        flip.setPosition(1);
        robot.lift.hang.setPosition(.6);
        //AutoTransitioner.transitionOnStop(this, "Lift Test");
        opencv.initialize();
        opencv.start();
        telemetry.addData("Initialized",mineralLocation);
        telemetry.update();
    }
    protected void initLoop(){
        switch(opencv.run()){
            case "LEFT":
                mineralLocation = MineralLocation.LEFT;
                break;
            case "RIGHT":
                mineralLocation = MineralLocation.RIGHT;
                break;
            case "CENTER":
                mineralLocation = MineralLocation.CENTER;
                break;
        }
        telemetry.addData("Initialized",mineralLocation);
        telemetry.update();
    }

    protected void descend(){
        telemetry.addData("Descending","Yeet");
        telemetry.update();
        //Reverse the lift for 1/5 a second, removes the passive hang, then brakes to slowly descend
        robot.lift.drive(.5);
        sleep(200);
        robot.lift.hang.setPosition(0);
        sleep(500);
        robot.lift.drive(0);
        //Maybe use gyro here?
        sleep(DESCEND_TIME*1000);
        //Needs to include delatching
    }

    protected void orientRobot(){
        //Drives forwards
        telemetry.addData("Orienting",MINERAL_ENCODER_SETUP);
        telemetry.update();
        robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER,
                MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<2&&opModeIsActive()) {
            telemetry.addData("Orienting",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    protected void hitMineralDepot(){
        //Depending on the mineral location, strafes sideways to it
        switch(mineralLocation) {
            case LEFT:
                robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                        -MINERAL_ENCODER_LEFT,MINERAL_ENCODER_LEFT,MINERAL_ENCODER_LEFT,-MINERAL_ENCODER_LEFT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){
                    telemetry.addData("Strafing",robot.mecanumDrive.frontLeft.getCurrentPosition());
                    telemetry.update();
                }
                break;
            case RIGHT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,
                        MINERAL_ENCODER_RIGHT,-MINERAL_ENCODER_RIGHT,-MINERAL_ENCODER_RIGHT,MINERAL_ENCODER_RIGHT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){
                    telemetry.addData("Strafing",robot.mecanumDrive.frontLeft.getCurrentPosition());
                    telemetry.update();
                }
                break;
        }
        //Hits mineral
        robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER,
                ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){
            telemetry.addData("Hitting",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    //protected void hitMineralCrater(){}

    protected void markerOrientDepot(long delay){
        //Turns to 45 degrees
        robot.mecanumDrive.useNoEncoders();
        timer.reset();
        do{
            //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
            turnPID.setPIDPower(-TURN_ANGLE,robot.mecanumDrive.getHeading(), true);
            double[] dp = {-turnPID.getPIDPower()*PID_SPEED, turnPID.getPIDPower()*PID_SPEED, -turnPID.getPIDPower()*PID_SPEED, turnPID.getPIDPower()*PID_SPEED};
            robot.mecanumDrive.drive(dp);
            telemetry.addData("Turning",robot.mecanumDrive.getHeading());
            telemetry.update();
        }while(turnPID.checkErrorLinear(TURN_TOLERANCE)&&opModeIsActive()&&timer.seconds()<PID_TIME);
        sleep(delay);
    }

    protected void SquareUp(){
        switch (mineralLocation){
            case RIGHT:
                squareUp(PLACE_RIGHT,SQUARE_TIME);
                break;
            case LEFT:
                squareUp(PLACE_LEFT,SQUARE_TIME);
                break;
            case CENTER:
                squareUp(PLACE_CENTER,SQUARE_TIME);
                break;
        }
    }

    protected void markerStrafe(){
        //Strafes to the right location
        switch(mineralLocation) {
            case LEFT:
                Strafe(MARKER_LEFT);
                break;
            case RIGHT:
                Strafe(MARKER_RIGHT);
                break;
            case CENTER:
                Strafe(MARKER_CENTER);
                break;
        }
        //Deposits team marker
        flip.setPosition(.35);
        sleep(SERVO_TIME);
    }

    //protected void markerOrientCrater(long delay){ }

    protected void park(long delay){
        //Parks and after .5 seconds brings the servo back up
        sleep(delay);
        //"Parks", by moving back enough for the team marker to slide off
        robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                -PARK,-PARK,-PARK,-PARK);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<PARK_TIME&&opModeIsActive()){
            telemetry.addData("Driving To Crater",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
            if(timer.seconds()>.25)
                flip.setPosition(1);
        }
    }
    protected void squareUp(int val,double Time){
        //Squares against wall
        robot.mecanumDrive.setDriveEncoders(.5*AUTO_DRIVE_POWER,.5*AUTO_DRIVE_POWER,.5*AUTO_DRIVE_POWER,.5*AUTO_DRIVE_POWER,
                val,val,val,val);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<Time&&opModeIsActive()){
            telemetry.addData("Strafing",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
        }
        robot.mecanumDrive.setDriveEncoders(-.5*AUTO_DRIVE_POWER,-.5*AUTO_DRIVE_POWER,-.5*AUTO_DRIVE_POWER,-.5*AUTO_DRIVE_POWER,
                -val,-val,-val,-val);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<Time&&opModeIsActive()){
            telemetry.addData("Going Forwards",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    protected void Strafe(int distance){
        robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                -distance,distance,distance,-distance);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<STRAFE_TIME&&opModeIsActive()){
            telemetry.addData("Strafing",robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
