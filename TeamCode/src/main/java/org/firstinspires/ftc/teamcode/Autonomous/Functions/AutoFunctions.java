package org.firstinspires.ftc.teamcode.Autonomous.Functions;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.HSV.LeftPropProcessor;
import org.firstinspires.ftc.teamcode.HSV.PropThreshold;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HSV.RightPropProcessor;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.util.CustomTypes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;


public class AutoFunctions {

    private static PropThreshold propThreshold;

    public static void startPosition (Servo clawHeadL, Servo clawHeadR){
        clawHeadL.setPosition(1);
        clawHeadR.setPosition(1);
    }



    public static boolean detectRed(int startingPos){
        if (startingPos == 1 || startingPos == 2) return false;
        return true;
    }
    public static int detectTeamElementleft(AprilTagProcessor tagProcessor, LeftPropProcessor leftPropProcessor, VisionPortal visionPortal){
        int objPos = 0;
        visionPortal.setProcessorEnabled(tagProcessor, false);
        visionPortal.setProcessorEnabled(leftPropProcessor, true);

        //Detects team elements location
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        objPos = getPropLocationleft(leftPropProcessor);
        visionPortal.setProcessorEnabled(leftPropProcessor, false);
        //visionPortal.setProcessorEnabled(tagProcessor, true);
        return objPos;
    }

    public static int detectTeamElementright(AprilTagProcessor tagProcessor, RightPropProcessor rightPropProcessor, VisionPortal visionPortal){
        int objPos = 0;
        visionPortal.setProcessorEnabled(tagProcessor, false);
        visionPortal.setProcessorEnabled(rightPropProcessor, true);

        //Detects team elements location
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        objPos = getPropLocationright(rightPropProcessor);
        visionPortal.setProcessorEnabled(rightPropProcessor, false);
        //visionPortal.setProcessorEnabled(tagProcessor, true);
        return objPos;
    }
public static int getPropLocationleft(LeftPropProcessor leftPropProcessor) {

    CustomTypes.PropLocation propLocation = leftPropProcessor.getPropLocation();
    if (propLocation == CustomTypes.PropLocation.LEFT) {
        return 1;
    } else if (propLocation == CustomTypes.PropLocation.MIDDLE) {
        return 2;
    } else {
        return 3;
    }

//    int leftCount = 0;
//    int middleCount = 0;
//    int rightCount = 0;
//    for (int i = 0; i < 50; i++) {
//        CustomTypes.PropLocation propLocation = leftPropProcessor.getPropLocation();
//        if (propLocation != null) {
//            switch (propLocation.location) {
//                case "left":
//                    leftCount++;
//                    break;
//                case "middle":
//                    middleCount++;
//                    break;
//                case "none":
//                    rightCount++;
//                    break;
//            }
//        }
//    }
//    if (leftCount > middleCount && leftCount > rightCount) return 1;
//    else if (middleCount > leftCount && middleCount > rightCount) return 2;
//    else return 3;

}
    public static int getPropLocationright(RightPropProcessor rightPropProcessor) {

        CustomTypes.PropLocation propLocation = rightPropProcessor.getPropLocation();
        if (propLocation == CustomTypes.PropLocation.LEFT) {
            return 1;
        } else if (propLocation == CustomTypes.PropLocation.MIDDLE) {
            return 2;
        } else {
            return 3;
        }

//        int leftCount = 0;
//        int middleCount = 0;
//        int rightCount = 0;
//        for (int i = 0; i < 50; i++) {
//            CustomTypes.PropLocation propLocation = rightPropProcessor.getPropLocation();
//            if (propLocation != null) {
//                switch (rightPropProcessor.getPropPosition()) {
//                    case "left":
//                        leftCount++;
//                        break;
//                    case "middle":
//                        middleCount++;
//                        break;
//                    case "none":
//                        rightCount++;
//                        break;
//                }
//            }
//        }
//        if (leftCount > middleCount && leftCount > rightCount) return 1;
//        else if (middleCount > leftCount && middleCount > rightCount) return 2;
//        else return 3;

    }

//    public static int getPropLocation(PropThreshold propThreshold) {
//
//        int leftCount = 0;
//        int middleCount = 0;
//        int rightCount = 0;
//        for (int i = 0; i < 50; i++) {
//            switch (propThreshold.getPropPosition()) {
//                case "left":
//                    leftCount++;
//                    break;
//                case "middle":
//                    middleCount++;
//                    break;
//                case "none":
//                    rightCount++;
//                    break;
//            }
//        }
//        if (leftCount > middleCount && leftCount > rightCount) return 1;
//        else if (middleCount > leftCount && middleCount > rightCount) return 2;
//        else return 3;
//
//    }

    public static int getTag(AprilTagProcessor tagProcessor,int startingPos) {
        int leftCount = 0;
        int middleCount = 0;
        int rightCount = 0;
        ArrayList<AprilTagDetection> detectionsList = tagProcessor.getDetections();
        if (detectionsList.size() != 0) {
            for (AprilTagDetection tag : detectionsList) {
                if (tag.id == 1 || tag.id == 4) {
                    leftCount += 1;
//                    telemetry.addData("location: ", "left");
                } else if (tag.id == 2 || tag.id == 5) {
                    middleCount += 1;
//                    telemetry.addData("location: ", "middle");
                } else if (tag.id == 3 || tag.id == 6) {
                    rightCount += 1;
//                    telemetry.addData("location: ", "right");
                } else {
                    middleCount += 1; // default is middle
//                    telemetry.addData("location: ", "not found");
                }
                break;
            }
            if (leftCount > middleCount && leftCount > rightCount) {
                if (startingPos < 4) return 1;
                else return 4;
            }
        } else if (rightCount > middleCount && rightCount > leftCount) {
            if (startingPos < 4) return 3;
            else return 5;
        } else {
            if (startingPos < 4) return 2;
            else return 4;
        }
        return 0; // Failed
    }


    public static void blueshiftShort(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) throws InterruptedException {
        Movement.linearSlides(900, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        sleep(100);
        Movement.right(114, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.right(10, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(3, telemetry, back_left, back_right, front_left, front_right);
        door.setPosition(1);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);

    }
    public static void blueshiftLong(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) throws InterruptedException {
        // in this code you should assume that the prop is on the left.
        Movement.linearSlides(900, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        sleep(100);
        Movement.right(240, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.right(10, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(10, telemetry, back_left, back_right, front_left, front_right);
        door.setPosition(1);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);
    }
    public static void redshiftShort(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) throws InterruptedException {

        Movement.linearSlides(900, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        sleep(100);
        Movement.left(96, telemetry, back_left, back_right, front_left, front_right); // reduced this distance from 114 to 96
        sleep(100);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.left(10, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(5, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        door.setPosition(1); // releases pixel onto backboard
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right); // why do we have this?


    }
    public static void redshiftLong(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) throws InterruptedException {
        // in this code you should assume that the prop is on the left.
        Movement.linearSlides(900, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        sleep(100);
        Movement.left(240, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.left(10, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);
        sleep(100);
        Movement.forward(10, telemetry, back_left, back_right, front_left, front_right);
        door.setPosition(1);
        Movement.forward(2, telemetry, back_left, back_right, front_left, front_right);
    }
}






