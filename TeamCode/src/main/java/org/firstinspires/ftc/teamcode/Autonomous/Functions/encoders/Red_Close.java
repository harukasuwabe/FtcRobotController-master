///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.Autonomous.Functions.encoders;
//
//import android.util.Size;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Autonomous.Functions.AutoFunctions;
//import org.firstinspires.ftc.teamcode.Autonomous.Functions.rightside;
//import org.firstinspires.ftc.teamcode.HSV.PropThreshold;
//import org.firstinspires.ftc.teamcode.HSV.RightPropProcessor;
//import org.firstinspires.ftc.teamcode.util.CustomTypes;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//
//
///*
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Auto_Red_Close", group="Test")
//public class Red_Close extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    int startingPos = 3; // Each corresponds to the quadrant one is starting at
//    /*
//     * 1 = Blue closer to stage
//     * 2 = Blue closer to parking
//     * 3 = Red closer to stage
//     * 4 = Red closer to parking
//     * */
//    int driveClawPos = 400;
//    int objPos = 0;
//     /*
//        Position of team object
//        1 = left
//        2 = center
//        3 = right
//        */
//
//    // purple pixel / spike mark: left
//    // yellow pixel / backdrop: right
//
//    private DcMotor back_left;
//    private DcMotor back_right;
//    private DcMotor front_left;
//    private DcMotor front_right;
//    private DcMotor linearSlideMotor_Left;
//    private DcMotor linearSlideMotor_Right;
//    private Servo door;
//    private Servo arm;
//    VisionPortal visionPortal;
//
//    BNO055IMU imu = null;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
//        params.angleUnit = BNO055IMU.AngleUnit.DEGREES; // sets angle degrees
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(params);
//
//        front_right = hardwareMap.get(DcMotor.class, "front_right");
//        front_left = hardwareMap.get(DcMotor.class, "front_left");
//        back_left = hardwareMap.get(DcMotor.class, "back_left");
//        back_right = hardwareMap.get(DcMotor.class, "back_right");
//        door = hardwareMap.get(Servo.class, "door");
//        arm = hardwareMap.get(Servo.class, "arm");
//        linearSlideMotor_Right = hardwareMap.dcMotor.get("linearSlideMotor_right");
//        linearSlideMotor_Left = hardwareMap.dcMotor.get("linearSlideMotor_left");
//        linearSlideMotor_Right.setDirection(DcMotor.Direction.REVERSE);
//        front_right.setDirection(DcMotor.Direction.REVERSE);
//        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        // Reversing needed motors
//
//
//        // Zero Power Behavior
//
//
//
//
//        telemetry.addData("Status", "updated");
//        telemetry.update();
//
//        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
//                .build();
//
//
//        RightPropProcessor rightPropProcessor = new RightPropProcessor();
//        rightPropProcessor.setDetectionColor(AutoFunctions.detectRed(startingPos));
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .addProcessors(tagProcessor,rightPropProcessor)
//                .build();
//
//        visionPortal.setProcessorEnabled(rightPropProcessor, true);
//
//        telemetry.addData("Detect Red", AutoFunctions.detectRed(startingPos));
//
//        waitForStart();
//        runtime.reset();
//        objPos = AutoFunctions.detectTeamElementright(tagProcessor,rightPropProcessor,visionPortal);
//        telemetry.addData("Obj Pos", objPos);
//        telemetry.addData("left perc, middle perc:", rightPropProcessor.getPercents());
//        telemetry.update();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//
//            if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.MIDDLE) {
//                rightside.centerProp(telemetry, back_left,back_right,front_left,front_right);
//                //AutoFunctions.redshiftShort(telemetry, back_left, back_right, front_left, front_right, linearSlideMotor_Left, linearSlideMotor_Right, arm, door);
//            }
//
//
//            else if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.LEFT) {
//                rightside.leftProp(telemetry, back_left, back_right, front_left, front_right);
//                //AutoFunctions.redshiftShort(telemetry, back_left, back_right, front_left, front_right, linearSlideMotor_Left, linearSlideMotor_Right, arm, door);
//
//            }
//            else  {
//                rightside.rightProp(telemetry, back_left, back_right, front_left, front_right);
//                //AutoFunctions.redshiftShort(telemetry, back_left, back_right, front_left, front_right, linearSlideMotor_Left, linearSlideMotor_Right, arm, door);
//
//
//            }
//        }
//    }
//}