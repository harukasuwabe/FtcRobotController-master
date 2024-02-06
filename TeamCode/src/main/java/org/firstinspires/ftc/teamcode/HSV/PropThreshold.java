package org.firstinspires.ftc.teamcode.HSV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropThreshold implements VisionProcessor {

    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();
    private static double threshold = 0.001;

    String outStr = "none"; //Set a default value in case vision does not work
    double leftPerc;
    double middlePerc;

    Rect LEFT_RECTANGLE;
    Rect MIDDLE_RECTANGLE;
    private boolean detectingRed = true;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        this.LEFT_RECTANGLE = new Rect(
                new Point(0,0),
                new Point(width/3, height)
        );

        this.MIDDLE_RECTANGLE = new Rect(
                new Point(width / 3, height),
                new Point(3 * width / 4, height)
        );
    }

    public void setDetectionColor(boolean detectRed) {
        if(detectRed) {
            this.detectingRed = true;
        } else {
            this.detectingRed = false;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if(this.detectingRed) {
            Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
            Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

            Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
            Scalar highHSVRedUpper = new Scalar(180, 255, 255);

            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);
            Core.bitwise_or(lowMat, highMat, finalMat);
        } else {
            Scalar blueHSVLower = new Scalar(85, 100, 20);
            Scalar blueHSVUpper = new Scalar(140, 255, 255);

            Core.inRange(testMat, blueHSVLower, blueHSVUpper, finalMat);
        }
        testMat.release();

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

        this.leftPerc = leftBox / LEFT_RECTANGLE.area() / 255;
        this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]
        this.middlePerc *= 2;

        if(leftPerc > middlePerc
                && leftPerc > threshold) {
            outStr = "left";
        } else if (middlePerc > leftPerc
                && middlePerc > threshold) {
            outStr = "middle";
        } else {
            outStr = "none";
        }

		/*This line should only be added in when you want to see your custom pipeline
		on the driver station stream, do not use this permanently in your code as
		you use the "frame" mat for all of your pipelines, such as April Tags*/
//		finalMat.copyTo(frame);
        Scalar redBorder = new Scalar(255, 0, 0);
        Scalar greenBorder = new Scalar(0, 255, 0);

        switch (outStr) {
            case "left":
                Imgproc.rectangle(frame, LEFT_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
            case "middle":
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
                break;
            case "none":
                Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
        }

        return frame;

//		return null; //You do not return the original mat anymore, instead return null

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public String getPropPosition(){  //Returns postion of the prop in a String
        return outStr;
    }

    public void setThreshold(double newThreshold) {
        threshold = newThreshold;
    }

    public double getRedThreshold() {
        return threshold;
    }

    public double[] getPercents() {
        return new double[]{leftPerc, middlePerc};
    }

}