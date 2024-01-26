

 package org.firstinspires.ftc.teamcode.Autonomous;
 import com.qualcomm.robotcore.hardware.DcMotor;

 import org.firstinspires.ftc.teamcode.Movement;
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
 @Autonomous(name = "BlueAutonomous_Distance", group = "Concept")

 public class Auto_Blue_Distant extends LinearOpMode {

     private DcMotor back_left;
     private DcMotor back_right;
     private DcMotor front_left;
     private DcMotor front_right;
     private DcMotor linearSlideMotor_Left;
     private DcMotor linearSlideMotor_Right;
     private Servo claw;
     private Servo door;


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
         double firstHalf = 300;
         init_Auto();

         double prop_x = 0;
         double id_x= 0;
         double maxExtention = 2400;
         int target_pos = 0;
         double far_blue =200;
         //initAprilTag();

         // Wait for the DS start button to be touched.
         telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
         telemetry.addData(">", "Touch Play to start OpMode");
         telemetry.update();

         waitForStart();
         if (opModeIsActive()) {



             if (prop_x == 0){
                 prop_x = detectProp();
                 detectProp();
             }


             telemetry.update();

             // Push telemetry to the Driver Station.

             // Save CPU resources; can resume streaming when needed.
             if (gamepad1.dpad_down) {
                 visionPortal.stopStreaming();
             }   else if (gamepad1.dpad_up) {
                 visionPortal.resumeStreaming();
             }

             // Share the CPU.
             sleep(20);

             if( prop_x>firstHalf) {
                 rightProp();
             }
             //id_x = leftID_x();
             //Movement.rotationRight(180, telemetry, back_left, back_right, front_left, front_right);

             //     linearSlideMotor_Right.setTargetPosition(target_pos);
             //     linearSlideMotor_Left.setTargetPosition(target_pos);
             //     linearSlideMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             //     linearSlideMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             //     linearSlideMotor_Right.setPower(1); // Adjust power as needed
             //     linearSlideMotor_Left.setPower(1); // Adjust power as needed
             //     claw.setPosition(1);
             //     door.setPosition(1);
             //     linearSlideMotor_Right.setTargetPosition(0);
             //     linearSlideMotor_Left.setTargetPosition(0);
             //     linearSlideMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             //     linearSlideMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             //     }

             if(prop_x < firstHalf && prop_x!= 0){
                 centerProp();
                 //     Movement.left(108,telemetry, back_left, back_right, front_left, front_right);
                 //     id_x = centerID_x();
                 //     Movement.rotationRight(180, telemetry, back_left, back_right, front_left, front_right);

             }
             else {
                 leftProp();
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
      * Initialize the TensorFlow Object Detection processor.
      */
     private void init_Auto() {

         // Create the TensorFlow processor by using a builder.
         tfod = new TfodProcessor.Builder()

                 // With the following lines commented out, the default TfodProcessor Builder
                 // will load the default model for the season. To define a custom model to load,
                 // choose one of the following:
                 //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                 //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                 //.setModelAssetName(TFOD_MODEL_ASSET)
                 .setModelFileName(TFOD_MODEL_FILE)

                 .setModelLabels(LABELS)
                 .build();
         aprilTag = new AprilTagProcessor.Builder()
                 .build();
         // Create the vision portal by using a builder.
         VisionPortal.Builder builder = new VisionPortal.Builder();

         // Set the camera (webcam vs. built-in RC phone camera).
         if (USE_WEBCAM) {
             builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
         } else {
             builder.setCamera(BuiltinCameraDirection.BACK);
         }

         // Choose a camera resolution. Not all cameras support all resolutions.
         //builder.setCameraResolution(new Size(640, 480));

         // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
         builder.enableLiveView(true);
         //builder.enableCameraMonitoring(true);
         builder.addProcessor(aprilTag);

         builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

         builder.setAutoStopLiveView(false);



         // Set and enable the processor.
         builder.addProcessor(tfod);

         // Build the Vision Portal, using the above settings.
         visionPortal = builder.build();

         // Set confidence threshold for TFOD recognitions, at any time.
         tfod.setMinResultConfidence(0.75f);

         // Disable or re-enable the TFOD processor at any time.
         //VisionPortal.setProcessorEnabled(tfod, true);


     }   // end method initTfod()




     /**
      * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
      */
     private double detectProp() {

         double x = 0;
         double y = 0;

         List<Recognition> currentRecognitions = tfod.getRecognitions();
         telemetry.addData("# Objects Detected", currentRecognitions.size());

         // Step through the list of recognitions and display info for each one.
         for (Recognition recognition : currentRecognitions) {
             x = (recognition.getLeft() + recognition.getRight()) / 2 ;
             y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

             telemetry.addData(""," ");
             telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
             telemetry.addData("- Position", "%.0f / %.0f", x, y);
             telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
         }   // end for() loop

         return x;

     }   // end method telemetryTfod()







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

     public double leftID_x(){
         double leftIDposition = 0;


         List<AprilTagDetection> currentDetections = aprilTag.getDetections();
         telemetry.addData("# AprilTags Detected", currentDetections.size());

         // Step through the list of detections and display info for each one.
         for (AprilTagDetection detection : currentDetections) {
             if (detection.id == 4 || detection.id == 1) {
                 telemetry.addLine("Left Detected");
                 leftIDposition = detection.ftcPose.x;
             }
             else {
                 Movement.right (10, telemetry, back_left, back_right, front_left, front_right);
                 if (detection.id == 4 || detection.id == 1) {
                     telemetry.addLine("Left Detected");
                     leftIDposition = detection.ftcPose.x;}
                 else {
                     telemetry.addLine("Left Detected");
                 }

             } //END OF ELSE
         }   // end for() loop
         return leftIDposition;

     }


     public double centerID_x(){
         double centerIDposition = 0;


         List<AprilTagDetection> currentDetections = aprilTag.getDetections();
         telemetry.addData("# AprilTags Detected", currentDetections.size());

         // Step through the list of detections and display info for each one.
         for (AprilTagDetection detection : currentDetections) {
             if (detection.id == 5 || detection.id == 2) {
                 telemetry.addLine("Center Detected");
                 centerIDposition = detection.ftcPose.x;
             }
             else {
                 Movement.right (10, telemetry, back_left, back_right, front_left, front_right);
                 if (detection.id == 5 || detection.id == 2) {
                     telemetry.addLine("Center Detected");
                     centerIDposition = detection.ftcPose.x;}
                 else {
                     telemetry.addLine("Center not Detected");
                 }

             } //END OF ELSE
         }   // end for() loop
         return centerIDposition;

     }


     public double rightID_x(){
         double rightIDposition = 0;


         List<AprilTagDetection> currentDetections = aprilTag.getDetections();
         telemetry.addData("# AprilTags Detected", currentDetections.size());

         // Step through the list of detections and display info for each one.
         for (AprilTagDetection detection : currentDetections) {
             if (detection.id == 6 || detection.id == 3) {
                 telemetry.addLine("Right Detected");
                 rightIDposition = detection.ftcPose.x;
             }
             else {
                 Movement.right (10, telemetry, back_left, back_right, front_left, front_right);
                 if (detection.id == 6 || detection.id == 3) {
                     telemetry.addLine("Right Detected");
                     rightIDposition = detection.ftcPose.x;

                 }
                 else {
                     telemetry.addLine("Right not Detected");
                 }

             } //END OF ELSE
         }  // end for() loop
         return rightIDposition;
     }
     public void leftProp(){
// in this code you should assume that the prop is on the left.
         Movement.backward(70,telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.backward(15, telemetry, back_left, back_right, front_left, front_right);
         Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.forward(60,telemetry, back_left, back_right, front_left, front_right);
         shiftLong();
//the above code may be wrong for the right

//scan
         //Movement.left(20, telemetry, back_left, back_right, front_left, front_right);
         //Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
//raise lift
/*
linnearSlideMotor_Right.setPower(1);
linnearSlideMotor_Left.setPower(1); */
//Movement.backward(35, telemetry, back_left, back_right, front_left, front_right);
//place pixel

     }

     public void centerProp(){
         //places pixel to the center and oes to the april tag position
         Movement.backward(65,telemetry, back_left, back_right, front_left, front_right);
         Movement.forward(55,telemetry, back_left, back_right, front_left, front_right);
         shiftLong();
     }

     public void rightProp(){
         //places pixel to the right and goes to the april tag position
         Movement.backward(70,telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.backward(5,telemetry, back_left, back_right, front_left, front_right);
         Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.forward(60,telemetry, back_left, back_right, front_left, front_right);
         Movement.left(10,telemetry, back_left, back_right, front_left, front_right);
         shiftLong();
     }//scan code thingy

     //raise lift and place pixel



     public void shiftLong(){
         // in this code you should assume that the prop is on the left.
         Movement.right( 180 , telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.left(20, telemetry, back_left, back_right, front_left, front_right);

         /*
        linearSlideMotor_Right.setPower(1);
        linearSlideMotor_Left.setPower(1); */
         Movement.backward(35, telemetry, back_left, back_right, front_left, front_right);
         //place pixel

     }



     public void testrightProp(){
         Movement.backward(75, telemetry, back_left, back_right, front_left, front_right);
         Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
         Movement.backward(25, telemetry, back_left, back_right, front_left, front_right);
         Movement.left(55, telemetry, back_left, back_right, front_left, front_right);
         Movement.backward(60, telemetry, back_left, back_right, front_left, front_right);
         Movement.right(160, telemetry, back_left, back_right, front_left, front_right);
         //raise lift and place pixel

     }

// end class
 }

