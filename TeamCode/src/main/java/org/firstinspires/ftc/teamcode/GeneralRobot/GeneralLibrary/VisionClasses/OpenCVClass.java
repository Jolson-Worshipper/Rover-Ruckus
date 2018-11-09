package org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.VisionClasses;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Size;

public class OpenCVClass extends Vision{
    private GoldDetector detector;

    public OpenCVClass(HardwareMap hwMap, Telemetry telem){
        super(hwMap,telem);
    }
    public void initialize(){
        detector = new GoldDetector(); // Create detector
        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
        detector.init(hardwaremap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set default detector settings
        // Optional tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void start(){
        detector.enable();
    }
    public String run(){
        if (detector.getScreenPosition().x < 200)
            return "LEFT";
        else if (detector.getScreenPosition().x < 350)
            return "CENTER";
        else if(detector.getScreenPosition().x > 350)
            return "RIGHT";
        else
            return "UNKNOWN";
    }
    public void close(){detector.disable();}
}
