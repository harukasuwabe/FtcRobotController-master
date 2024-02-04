/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HSV.PropThreshold;
import org.firstinspires.ftc.teamcode.Autonomous.AutoFunctions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="redCamTest", group="Test")
public class redCamTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    int startingPos = 3; // Each corresponds to the quadrant one is starting at
    /*
     * 1 = Blue closer to stage
     * 2 = Blue closer to parking
     * 3 = Red closer to stage
     * 4 = Red closer to parking
     * */
    int driveClawPos = 400;
    int objPos = 0;
     /*
        Position of team object
        1 = left
        2 = center
        3 = right
        */

    // purple pixel / spike mark: left
    // yellow pixel / backdrop: right

    private DcMotorEx motorFL = null;
    private DcMotorEx motorFR = null;
    private DcMotorEx motorBL = null;
    private DcMotorEx motorBR = null;

    private DcMotorEx scissor = null;
    VisionPortal visionPortal;

    Servo clawL;
    Servo clawR;
    Servo clawHeadL;
    Servo clawHeadR;

    BNO055IMU imu = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES; // sets angle degrees
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");

        scissor = hardwareMap.get(DcMotorEx.class,"scissor");

        scissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL= hardwareMap.get(Servo.class, "clawL");
        clawR= hardwareMap.get(Servo.class, "clawR");
        clawHeadL= hardwareMap.get(Servo.class, "clawHeadL");
        clawHeadR= hardwareMap.get(Servo.class, "clawHeadL");

        // Reversing needed motors
        motorBR.setDirection(DcMotorEx.Direction.FORWARD);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setDirection(DcMotorEx.Direction.REVERSE);
        scissor.setDirection(DcMotorEx.Direction.REVERSE);

        // Zero Power Behavior
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        scissor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        scissor.setPositionPIDFCoefficients(1);

        telemetry.addData("Status", "updated");
        telemetry.update();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        PropThreshold propThreshold = new PropThreshold();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(tagProcessor,propThreshold)
                .build();


        telemetry.addData("Detect Red", AutoFunctions.detectRed(startingPos));
        propThreshold.setDetectionColor(AutoFunctions.detectRed(startingPos));

        waitForStart();
        runtime.reset();
        objPos = AutoFunctions.detectTeamElement(tagProcessor,propThreshold,visionPortal);
        telemetry.addData("Obj Pos", objPos);
        telemetry.addData("left perc, middle perc:", propThreshold.getPercents());
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }
}