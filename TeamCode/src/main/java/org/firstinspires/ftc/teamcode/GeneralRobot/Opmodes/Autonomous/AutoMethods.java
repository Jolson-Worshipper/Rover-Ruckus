package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.PIDClass;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;

public abstract class AutoMethods extends LinearOpMode {
    protected enum AutoStates {
        HANG_DESCEND,
        ORIENT_ROBOT,
        HIT_MINERAL,
        PLACE_MARKER_ORIENT,
        PLACE_MARKER,
        CRATER_PARK,
        STOP
    }
    protected enum MineralLocation {
        LEFT,
        CENTER,
        RIGHT
    }
    public enum StartLocation {
        CRATER,
        DEPOT,
        NONE
    }

    protected StartLocation startLocation = StartLocation.NONE;
    private ElapsedTime timer;
    protected AutoStates autoStates = TestAuto.AutoStates.HIT_MINERAL;
    private MineralLocation mineralLocation = TestAuto.MineralLocation.LEFT;
    protected Robot robot;
    private PIDClass turnPID;
    private double forwardAngle;
    protected final int LIFT_ENCODER_AMOUNT = 0;
    protected final double TURN_TOLERANCE = .1;
    protected final double ORIENT_ANGLE = 50;
    protected final double AUTO_DRIVE_POWER = .3;
    protected final int MINERAL_ENCODER_SETUP = 50;
    protected final int MINERAL_ENCODER_LEFT = 50;
    protected final int MINERAL_ENCODER_CENTER = 50;
    protected final int MINERAL_ENCODER_RIGHT = 50;
    protected final int MARKER_ENCODER_LEFT_WALL = 50;
    protected final int MARKER_ENCODER_CENTER_WALL = 50;
    protected final int MARKER_ENCODER_RIGHT_WALL = 50;
    protected final int MARKER_ENCODER_LEFT_PLACE = 50;
    protected final int MARKER_ENCODER_CENTER_PLACE = 50;
    protected final int MARKER_ENCODER_RIGHT_PLACE = 50;
    protected final int PARK_ENCODER_LEFT = 50;
    protected final int PARK_ENCODER_CENTER = 50;
    protected final int PARK_ENCODER_RIGHT = 50;

    public abstract void runOpMode()throws InterruptedException;


    protected void initOnce(){
        timer = new ElapsedTime();
        robot = new Robot(hardwareMap, gamepad1, telemetry);
        robot.initializeRobot();
        robot.initializeVision();
        robot.mecanumDrive.restistOnBrake();
        turnPID = new PIDClass(.5,0,0);
    }

    protected void descend(){
        robot.lift.descend(LIFT_ENCODER_AMOUNT);
        while(robot.lift.isBusy())
            idle();
        robot.mecanumDrive.moveToDelatch();
        while(robot.mecanumDrive.isBusy())
            idle();
        autoStates = TestAuto.AutoStates.ORIENT_ROBOT;
    }

    protected void orientRobot(){
        robot.vuforiaClass.initialize();
        do{
            robot.vuforiaClass.run();
            double[] drivePowers = {.1, -.1, .1, -.1};
            robot.mecanumDrive.drive(drivePowers);
        }while(!robot.vuforiaClass.getVisible());
        robot.mecanumDrive.brake();
        forwardAngle = robot.vuforiaClass.getAngle();
        do{
            robot.mecanumDrive.updateGyro();
            //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
            turnPID.setPIDPower(ORIENT_ANGLE-forwardAngle,robot.mecanumDrive.getHeading(), true);
            double[] drivePowers = {turnPID.getPIDPower(), -turnPID.getPIDPower(), turnPID.getPIDPower(), -turnPID.getPIDPower()};
            robot.mecanumDrive.drive(drivePowers);
        }while(turnPID.checkErrorLinear(TURN_TOLERANCE));
        //Drives forward, will hopefully do sturn after we get a basic auto done...
        robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER, AUTO_DRIVE_POWER,
                MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP, MINERAL_ENCODER_SETUP);
        while(robot.mecanumDrive.isBusy()) {
            idle();
        }
        autoStates = AutoStates.HIT_MINERAL;
    }

    protected void hitMineralDepot(){
        switch(mineralLocation) {
            case LEFT:
                //Consider Sturn
                //Currently disabled because I can't tell if the first part works right
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 90, MINERAL_ENCODER_LEFT);
                while(robot.mecanumDrive.isBusy())
                    idle();
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MINERAL_ENCODER_CENTER);
                break;
            case RIGHT:
                //Consider Sturn
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 270, MINERAL_ENCODER_RIGHT);
                while(robot.mecanumDrive.isBusy())
                    idle();
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MINERAL_ENCODER_CENTER);
                break;
            case CENTER:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MINERAL_ENCODER_CENTER);
                break;
        }
        while(robot.mecanumDrive.isBusy()) {
            idle();
        }
        autoStates = AutoStates.PLACE_MARKER_ORIENT;
    }

    protected void hitMineralCrater(){}

    protected void markerOrientDepot(long delay){
        do{
            robot.mecanumDrive.updateGyro();
            //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
            turnPID.setPIDPower(forwardAngle,robot.mecanumDrive.getHeading(), true);
            double[] drivePowers = {turnPID.getPIDPower(), -turnPID.getPIDPower(), turnPID.getPIDPower(), -turnPID.getPIDPower()};
            robot.mecanumDrive.drive(drivePowers);
        }while(turnPID.checkErrorLinear(TURN_TOLERANCE));
        sleep(delay);
        switch(mineralLocation) {
            case LEFT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 90, MARKER_ENCODER_LEFT_WALL);
                while(robot.mecanumDrive.isBusy())
                    idle();
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MARKER_ENCODER_LEFT_PLACE);
                break;
            case RIGHT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 90, MARKER_ENCODER_RIGHT_WALL);
                while(robot.mecanumDrive.isBusy())
                    idle();
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MARKER_ENCODER_RIGHT_PLACE);
                break;
            case CENTER:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 90, MARKER_ENCODER_CENTER_WALL);
                while(robot.mecanumDrive.isBusy())
                    idle();
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 0, MARKER_ENCODER_CENTER_PLACE);
                break;
        }
        while(robot.mecanumDrive.isBusy()){
            idle();
        }
        autoStates = AutoStates.PLACE_MARKER;
    }

    protected void markerOrientCrater(long   delay){ }

    protected void park(long delay){
        sleep(delay);
        switch(mineralLocation) {
            case LEFT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 180, PARK_ENCODER_LEFT);
                break;
            case RIGHT:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 180, PARK_ENCODER_RIGHT);
                break;
            case CENTER:
                robot.mecanumDrive.setDriveEncoders(AUTO_DRIVE_POWER, 180, PARK_ENCODER_CENTER);
                break;
        }
        while(robot.mecanumDrive.isBusy()){
            idle();
        }
        autoStates = AutoStates.STOP;
    }
}
