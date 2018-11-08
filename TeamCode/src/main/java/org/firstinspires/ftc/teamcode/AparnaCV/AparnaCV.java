package org.firstinspires.ftc.teamcode.AparnaCV;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.EnderCV.OpenCVPipeline;

/**
 * Created by Victo on 9/10/2018.
 */

public abstract class AparnaCV extends OpenCVPipeline{

    public abstract Mat process(Mat input);
    public abstract void useDefaults();

    private List<MaxAreaScorer> scorers = new ArrayList<>();
    private Size initSize;
    private Size adjustedSize;
    private Mat workingMat = new Mat();
    public double maxDifference = 10;
    public double downscale = 0.5;
    public Size   downscaleResolution = new Size(640, 480);
    public boolean useFixedDownscale = true;
    protected String detectorName = "AparnaCV Detector";

    public AparnaCV(){

    }

    public void addScorer(MaxAreaScorer newScorer){
        scorers.add(newScorer);
    }

    public double calculateScore(Mat input){
        double totalScore = 0;

        for(MaxAreaScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }



    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        initSize = rgba.size();

        if(useFixedDownscale){
            adjustedSize = downscaleResolution;
        }else{
            adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
        }

        rgba.copyTo(workingMat);

        if(workingMat.empty()){
            return rgba;
        }
        Imgproc.resize(workingMat, workingMat,adjustedSize); // Downscale
        Imgproc.resize(process(workingMat),workingMat,getInitSize()); // Process and scale back to original size for viewing
        //Print Info
        Imgproc.putText(workingMat,"AparnaCV 2018.2 " + detectorName + ": " + getAdjustedSize().toString() + " - " + "Balanced Speed" ,new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;
    }

    public Size getInitSize() {
        return initSize;
    }

    public Size getAdjustedSize() {
        return adjustedSize;
    }

    public void setAdjustedSize(Size size) { this.adjustedSize = size; }
}
