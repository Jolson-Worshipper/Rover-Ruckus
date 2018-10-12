package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.GeneralRobot.SubsystemClasses.Robot;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.PIDClass;

@Autonomous(name="Preliminary Auto Test!",group="Auto")
public class TestAuto extends LinearOpMode{
    private enum AutoStates {
        HANG_DESCEND,
        ORIENT_ROBOT,
        HIT_MINERAL,
        PLACE_MARKER_ORIENT,
        PLACE_MARKER,
        CRATER_PARK,
        STOP
    }
    private enum MineralLocation {
        LEFT,
        CENTER,
        RIGHT
    }
    private ElapsedTime timer;
    private AutoStates autoStates = AutoStates.HIT_MINERAL;
    private MineralLocation mineralLocation = MineralLocation.LEFT;
    private Robot robot;
    private PIDClass turnPID;
    private double forwardAngle;
    public final int LIFT_ENCODER_AMOUNT = 0;
    public final double TURN_TOLERANCE = .1;
    public final double ORIENT_ANGLE = 50;
    public final double AUTO_DRIVE_POWER = .3;
    public final int MINERAL_ENCODER_SETUP = 50;
    public final int MINERAL_ENCODER_LEFT = 50;
    public final int MINERAL_ENCODER_CENTER = 50;
    public final int MINERAL_ENCODER_RIGHT = 50;
    public final int MARKER_ENCODER_LEFT_WALL = 50;
    public final int MARKER_ENCODER_CENTER_WALL = 50;
    public final int MARKER_ENCODER_RIGHT_WALL = 50;
    public final int MARKER_ENCODER_LEFT_PLACE = 50;
    public final int MARKER_ENCODER_CENTER_PLACE = 50;
    public final int MARKER_ENCODER_RIGHT_PLACE = 50;
    public final int PARK_ENCODER_LEFT = 50;
    public final int PARK_ENCODER_CENTER = 50;
    public final int PARK_ENCODER_RIGHT = 50;

    public void runOpMode() throws InterruptedException{
        timer = new ElapsedTime();
        robot = new Robot(hardwareMap, gamepad1, telemetry);
        robot.mecanumDrive.restistOnBrake();
        turnPID = new PIDClass(.5,0,0);
        /*
        Scan OpenCV Stuff
         */
        waitForStart();
        //Run main auto
        while(opModeIsActive()){
            robot.mecanumDrive.updateGyro();
            switch(autoStates){
                case HANG_DESCEND:
                    robot.lift.descend(LIFT_ENCODER_AMOUNT);
                    while(robot.lift.isBusy())
                        idle();
                    robot.mecanumDrive.moveToDelatch();
                    while(robot.mecanumDrive.isBusy())
                        idle();
                    autoStates = AutoStates.ORIENT_ROBOT;
                    break;
                case ORIENT_ROBOT:
                    forwardAngle = robot.vuforiaClass.getAngle();
                    do{
                        robot.mecanumDrive.updateGyro();
                        //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
                        turnPID.setPIDPower(ORIENT_ANGLE-forwardAngle,robot.mecanumDrive.heading, true);
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
                    break;
                case HIT_MINERAL:
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
                    break;
                case PLACE_MARKER_ORIENT:
                    do{
                        robot.mecanumDrive.updateGyro();
                        //Will have to tune this part, not exactly sure what value vuforia actually gives (ex. is 0 facing straight forward?)
                        turnPID.setPIDPower(forwardAngle,robot.mecanumDrive.heading, true);
                        double[] drivePowers = {turnPID.getPIDPower(), -turnPID.getPIDPower(), turnPID.getPIDPower(), -turnPID.getPIDPower()};
                        robot.mecanumDrive.drive(drivePowers);
                    }while(turnPID.checkErrorLinear(TURN_TOLERANCE));
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
                    break;
                    //WIP
                case PLACE_MARKER:
                    //idk how we are placing the marker yet yikes
                    autoStates = AutoStates.CRATER_PARK;
                    break;
                //WIP
                case CRATER_PARK:
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
                    break;
                case STOP:
                    stop();
                    break;
                }
            }
        }
}
