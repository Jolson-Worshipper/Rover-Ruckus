package org.firstinspires.ftc.teamcode.GeneralRobot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.AutoTransitioner;
import org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.GamepadClasses.GamepadClass;

@Autonomous(name="Preliminary Auto Test!!!",group="Auto")
public class TestAuto extends AutoMethods{
    private long delay1=0;
    private long delay2=0;
    private boolean yTog,xTog,rbTog,lbTog=false;
    GamepadClass gPad = new GamepadClass(gamepad1);

    public void runOpMode() throws InterruptedException{
        initOnce();
        while(!isStarted()){
            gPad.update();
            if(gPad.A.getVal())
                startLocation = StartLocation.CRATER;
            else if(gPad.B.getVal())
                startLocation = StartLocation.DEPOT;
            if(gPad.LB.getToggle()!=lbTog) {
                delay1 += .5;
                lbTog = !lbTog;
            }
            else if(gPad.RB.getToggle()!=rbTog) {
                delay1 -= .5;
                rbTog = !rbTog;
            }
            if(gPad.X.getToggle()!=xTog) {
                delay2 += .5;
                xTog = !xTog;
            }
            else if(gPad.Y.getToggle()!=yTog) {
                delay2 -= .5;
                yTog = !yTog;
            }
            telemetry.addData("Start Location",startLocation);
            telemetry.addData("First Delay",delay1);
            telemetry.addData("Second Delay",delay2);
            telemetry.update();
        }
        AutoTransitioner.transitionOnStop(this, "FC Teleop");
        waitForStart();
        startLocation = StartLocation.DEPOT;
        //Run main auto
        while(opModeIsActive()){
            switch(autoStates){
                case HANG_DESCEND:
                   descend();
                    break;
                case ORIENT_ROBOT:
                    orientRobot();
                    break;
                case HIT_MINERAL:
                    if(startLocation == StartLocation.CRATER)
                        hitMineralCrater();
                    else if(startLocation == StartLocation.DEPOT)
                        hitMineralDepot();
                    break;
                case PLACE_MARKER_ORIENT:
                    if(startLocation == StartLocation.CRATER)
                        markerOrientCrater(delay1);
                    else if(startLocation == StartLocation.DEPOT)
                        markerOrientDepot(delay1);
                    break;
                    //WIP
                case PLACE_MARKER:
                    //idk how we are placing the marker yet yikes
                    autoStates = AutoStates.CRATER_PARK;
                    break;
                //WIP
                case CRATER_PARK:
                    park(delay2);
                    break;
                case STOP:
                    stop();
                    break;
            }
        }
    }
}
