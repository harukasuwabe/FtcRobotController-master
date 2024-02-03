

package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpenCv.Open.cv.TeamElementSubsystem;

import org.firstinspires.ftc.teamcode.OpenCv.Open.cv.SplitAveragePipeline;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "BlueCloseVision", group = "Concept")

public class Auto_BlueClose extends LinearOpMode {

    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor linearSlideMotor_Left;
    private DcMotor linearSlideMotor_Right;
    private Servo door;
    private Servo arm;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "model_20240117_115245.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue_prop",
    };


    @Override
    public void runOpMode() {

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        door = hardwareMap.get(Servo.class, "door");
        arm = hardwareMap.get(Servo.class, "arm");
        double firstHalf = 300;

        double prop_x = 0;
        double id_x = 0;
        double maxExtention = 2400;
        int target_pos = 0;
        double far_blue = 200;
        //initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {


            telemetry.update();

            // Push telemetry to the Driver Station.

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);

            Movement.linearSlides(100, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);


            if (TeamElementSubsystem.elementDetection(telemetry) == 1) {
                leftProp();
            }


            if (TeamElementSubsystem.elementDetection(telemetry) == 2) {
                centerProp();
            }
            if (TeamElementSubsystem.elementDetection(telemetry) == 3) {
                rightProp();

            } else {
                rightProp();
                //leftIDposition
                Movement.right(240, telemetry, back_left, back_right, front_left, front_right);

                //     rightProp();
                //     Movement.left(108,telemetry, back_left, back_right, front_left, front_right);
                //     id_x = rightID_x();
                //     Movement.rotationRight(180, telemetry, back_left, back_right, front_left, front_right);

            }


        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        // end method runOpMode()
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    private void detectAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public double leftID_x() {
        double leftIDposition = 0;


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 4 || detection.id == 1) {
                telemetry.addLine("Left Detected");
                leftIDposition = detection.ftcPose.x;
            } else {
                Movement.right(10, telemetry, back_left, back_right, front_left, front_right);
                if (detection.id == 4 || detection.id == 1) {
                    telemetry.addLine("Left Detected");
                    leftIDposition = detection.ftcPose.x;
                } else {
                    telemetry.addLine("Left Detected");
                }

            } //END OF ELSE
        }   // end for() loop
        return leftIDposition;

    }


    public double centerID_x() {
        double centerIDposition = 0;


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 5 || detection.id == 2) {
                telemetry.addLine("Center Detected");
                centerIDposition = detection.ftcPose.x;
            } else {
                Movement.right(10, telemetry, back_left, back_right, front_left, front_right);
                if (detection.id == 5 || detection.id == 2) {
                    telemetry.addLine("Center Detected");
                    centerIDposition = detection.ftcPose.x;
                } else {
                    telemetry.addLine("Center not Detected");
                }

            } //END OF ELSE
        }   // end for() loop
        return centerIDposition;

    }


    public double rightID_x() {
        double rightIDposition = 0;


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 6 || detection.id == 3) {
                telemetry.addLine("Right Detected");
                rightIDposition = detection.ftcPose.x;
            } else {
                Movement.right(10, telemetry, back_left, back_right, front_left, front_right);
                if (detection.id == 6 || detection.id == 3) {
                    telemetry.addLine("Right Detected");
                    rightIDposition = detection.ftcPose.x;

                } else {
                    telemetry.addLine("Right not Detected");
                }

            } //END OF ELSE
        }  // end for() loop
        return rightIDposition;
    }

    public void leftProp() {
// in this code you should assume that the prop is on the left.
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        //Movement.backward(5,telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(65, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);
        shiftShort();


    }

    public void centerProp() {
        //places pixel to the center and oes to the april tag position
        Movement.backward(65, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(55, telemetry, back_left, back_right, front_left, front_right);
        shiftShort();
    }

    public void rightProp() {
        //places pixel to the right and goes to the april tag position
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.backward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(60, telemetry, back_left, back_right, front_left, front_right);
        shiftShort();
    }//scan code thingy

    //raise lift and place pixel


    public void shiftShort() {
        Movement.right(61, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(20, telemetry, back_left, back_right, front_left, front_right);
        Movement.linearSlides(1240, telemetry, linearSlideMotor_Left, linearSlideMotor_Right);
        arm.setPosition(-1);
        door.setPosition(-1);

         /*
        linearSlideMotor_Right.setPower(1);
        linearSlideMotor_Left.setPower(1); */
        //Movement.backward(35, telemetry, back_left, back_right, front_left, front_right);
        //place pixel

    }
}


