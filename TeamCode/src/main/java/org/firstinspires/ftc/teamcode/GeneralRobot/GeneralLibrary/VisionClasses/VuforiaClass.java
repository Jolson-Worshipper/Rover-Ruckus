package org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class VuforiaClass extends Vision{

    //Makes private instances of these so stuff from initialize() can be used in run()
    private CloseableVuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relictrackable;


    public VuforiaClass(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap,telemetry);
    }

    //Sets up vuforia up to starting to scan
    public void initialize(){
        int cameraMonitorViewId = hardwaremap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwaremap.appContext.getPackageName());
        CloseableVuforiaLocalizer.Parameters parameters = new CloseableVuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Our vuforia code
        parameters.vuforiaLicenseKey = "Aa07QPX/////AAAAGT4IBGftwkAmodz5uX1NKehqWSuZYAizMXyJgDjbMQz+h5mPdKPRRA9id11R2ad9e3w3E6aS1Nep0aXgwwqRtAAmh6tizyQQZRM5qF+foaOh9zbuyAis/ANMODT0X5fAo3J6DqPNlOT9Es04EMKR5rIGhrb91rn3X+ferq2phtQ/PhQGHt44rkhNXSI1OV2GaY4BErnIgSktLZB6bWf49Jd3RtnybC9BfsuOv/2re0pEiGAiF+GyTV5pvuyVVFXFMKaiIR+aDe8qBpKV5z+ZUIWUC+z989ERqh9SKWdfJkOJt6glYFx/fEy3o4g8HwYfVbU+xU1fxufN+M3A2uZZaSSowVbbDDgr9CGxSd6/Dskg";
        //We want to use the front camera
        parameters.cameraDirection = CloseableVuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = new CloseableVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relictrackable = relicTrackables.get(0);
        relictrackable.setName("relicVuMark");
    }


    public String run() {
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relictrackable);

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addData("LEFT", vuMark);
            telemetry.update();
            return "LEFT";
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("RIGHT", vuMark);
            telemetry.update();
            return "RIGHT";
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("CENTER", vuMark);
            telemetry.update();
            return "CENTER";
        } else {
            telemetry.addData("UNKNOWN", vuMark);
            telemetry.update();
            return "UNKNOWN";
        }
    }

    public void close(){
        vuforia.close();
    }
}
