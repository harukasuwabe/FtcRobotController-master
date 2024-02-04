package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.HSV.PropThreshold;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;
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

    public static int detectTeamElement(AprilTagProcessor tagProcessor, PropThreshold propThreshold, VisionPortal visionPortal){
        int objPos = 0;
        visionPortal.setProcessorEnabled(tagProcessor, false);
        visionPortal.setProcessorEnabled(propThreshold, true);

        //Detects team elements location
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        objPos = getPropLocation(propThreshold);
        visionPortal.setProcessorEnabled(propThreshold, false);
        //visionPortal.setProcessorEnabled(tagProcessor, true);
        return objPos;
    }

    public static int getPropLocation(PropThreshold propThreshold) {

        int leftCount = 0;
        int middleCount = 0;
        int rightCount = 0;
        for (int i = 0; i < 50; i++) {
            switch (propThreshold.getPropPosition()) {
                case "left":
                    leftCount++;
                    break;
                case "middle":
                    middleCount++;
                    break;
                case "none":
                    rightCount++;
                    break;
            }
        }
        if (leftCount > middleCount && leftCount > rightCount) return 1;
        else if (middleCount > leftCount && middleCount > rightCount) return 2;
        else return 3;
    }

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


    public void blueshiftShort(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) {
        Movement.right(61, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(20, telemetry, back_left, back_right, front_left, front_right);
        Movement.linearSlides(1240, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        door.setPosition(-1);


    }
    public void bluehiftLong(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door){
        // in this code you should assume that the prop is on the left.
        Movement.right( 180 , telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(20, telemetry, back_left, back_right, front_left, front_right);

        Movement.linearSlides(1240, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        door.setPosition(-1);
    }
    public void redshiftShort(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door) {
        Movement.left(61, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.left(20, telemetry, back_left, back_right, front_left, front_right);
        Movement.linearSlides(1240, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        door.setPosition(-1);


    }
    public void redshiftLong(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right, DcMotor linearSlideMotor_Left, DcMotor linearSlideMotor_Right, Servo arm, Servo door){
        // in this code you should assume that the prop is on the left.
        Movement.left( 180 , telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.left(20, telemetry, back_left, back_right, front_left, front_right);

        Movement.linearSlides(1240, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        door.setPosition(-1);
    }
}






