package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.PIDClass;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

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
    //Use this once we get 2 autos
    protected StartLocation startLocation = StartLocation.DEPOT;
    //Change this to RIGHT,LEFT,CENTER, to test the 3 mineral locations
    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    protected final double KP = .0225;
    protected final double KI = .001;
    protected final double KD = 0;
    //Tolerance that PID uses before accepting its angle
    protected final double TURN_TOLERANCE = 1;
    //Speeds of the robot
    protected final double AUTO_DRIVE_POWER = .5;
    protected final double PID_SPEED = .4;
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
    //Distance to go forwards after strafing to team marker spot
    protected final int PLACE_LEFT = 300;
    protected final int PLACE_CENTER = 200;
    protected final int PLACE_RIGHT = 0;
    //Park distance
    protected final int PARK = 4000;
    //Time to use encoders at the start
    protected final double ENCODER_TIME = 2;
    //Time to rotate
    protected final double PID_TIME = 3;
    //Time to descend
    protected final long DESCEND_TIME = 15000;
    //Time to park/get over there
    protected final double PARK_TIME = 10;
    //Time to bring a servo down
    protected final long SERVO_TIME = 300;
    //TIme to strafe sideways to deposit marker
    protected final double STRAFE_TIME = 5;

    public void runOpMode() throws InterruptedException{
        flip = hardwareMap.servo.get("flip");
        initOnce();
        //Inits team marker+passive hang
        flip.setPosition(1);
        robot.lift.hang.setPosition(.6);
        waitForStart();
        //descend();
        //brings the robot out of the park zone
        orientRobot();
        //strafes to then hits the mineral
        hitMineralDepot();
        //rotates, strafes sideways, deposits team marker
        markerOrientDepot(0);
        //drives back and parks
        park(0);
        requestOpModeStop();
    }
    
    protected void initOnce(){
        timer = new ElapsedTime();
        robot = new Robot(hardwareMap, gamepad1, telemetry);
        robot.mecanumDrive.restistOnBrake();
        turnPID = new PIDClass(KP,KI,KD);
    }
    //protected void initLoop(){}

    protected void descend(){
        //Reverse the lift for 1/5 a second, removes the passive hang, then brakes to slowly descend
        robot.lift.drive(.5);
        sleep(200);
        robot.lift.hang.setPosition(0);
        sleep(500);
        robot.lift.drive(0);
        //Maybe use gyro here?
        sleep(DESCEND_TIME*1000);
    }

    protected void orientRobot(){
        //Drives forwards
        robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER,
                MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<2&&opModeIsActive()) {
            idle();
        }
    }

    protected void hitMineralDepot(){
        //Depending on the mineral location, strafes sideways to it
        switch(mineralLocation) {
            case LEFT:
                robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                        -MINERAL_ENCODER_LEFT,MINERAL_ENCODER_LEFT,MINERAL_ENCODER_LEFT,-MINERAL_ENCODER_LEFT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
                break;
            case RIGHT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,
                        MINERAL_ENCODER_RIGHT,-MINERAL_ENCODER_RIGHT,-MINERAL_ENCODER_RIGHT,MINERAL_ENCODER_RIGHT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
                break;
        }
        //Hits mineral
        robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER,
                ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL, ENCODER_HIT_MINERAL);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
        robot.mecanumDrive.brake();
    }

    //protected void hitMineralCrater(){}

    protected void markerOrientDepot(long delay){
        //Turns to 45 degrees
        robot.mecanumDrive.useNoEncoders();
        timer.reset();
        do{
            //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
            turnPID.setPIDPower(-45,robot.mecanumDrive.getHeading(), true);
            double[] dp = {-turnPID.getPIDPower()*PID_SPEED, turnPID.getPIDPower()*PID_SPEED, -turnPID.getPIDPower()*PID_SPEED, turnPID.getPIDPower()*PID_SPEED};
            robot.mecanumDrive.drive(dp);
        }while(turnPID.checkErrorLinear(TURN_TOLERANCE)&&opModeIsActive()&&timer.seconds()<PID_TIME);
        sleep(delay);
        //Strafes sideways depending on location, then strafes forwards if needed
        switch(mineralLocation) {
            case LEFT:
                robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                        -MARKER_LEFT,MARKER_LEFT,MARKER_LEFT,-MARKER_LEFT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<STRAFE_TIME&&opModeIsActive()){}
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,
                        PLACE_LEFT,PLACE_LEFT,PLACE_LEFT,PLACE_LEFT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
                break;
            case RIGHT:
                robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                        -MARKER_RIGHT,MARKER_RIGHT,MARKER_RIGHT,-MARKER_RIGHT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<STRAFE_TIME&&opModeIsActive()){}
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,
                        PLACE_RIGHT,PLACE_RIGHT,PLACE_RIGHT,PLACE_RIGHT);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
                break;
            case CENTER:
                robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                        -MARKER_CENTER,MARKER_CENTER,MARKER_CENTER,-MARKER_CENTER);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<STRAFE_TIME&&opModeIsActive()){}
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,AUTO_DRIVE_POWER,
                        PLACE_CENTER,PLACE_CENTER,PLACE_CENTER,PLACE_CENTER);
                timer.reset();
                while(robot.mecanumDrive.isBusy()&&timer.seconds()<ENCODER_TIME&&opModeIsActive()){}
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
        robot.mecanumDrive.setDriveEncoders(-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,-AUTO_DRIVE_POWER,
                -PARK,-PARK,-PARK,-PARK);
        timer.reset();
        while(robot.mecanumDrive.isBusy()&&timer.seconds()<PARK_TIME&&opModeIsActive()){
            if(timer.seconds()>.5)
                flip.setPosition(1);
        }
    }
}
