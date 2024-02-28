package org.firstinspires.ftc.teamcode.Autonomous.Functions.roadRunner;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.HSV.RightPropProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.CustomTypes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="red_Close_RR", group="roadRunner")
public class redClose extends LinearOpMode {

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

    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor linearSlideMotor_Left;
    private DcMotor linearSlideMotor_Right;
    private Servo door;
    private Servo arm;
    VisionPortal visionPortal;

    BNO055IMU imu = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES; // sets angle degrees
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        door = hardwareMap.get(Servo.class, "door");
        arm = hardwareMap.get(Servo.class, "arm");
        linearSlideMotor_Right = hardwareMap.dcMotor.get("linearSlideMotor_right");
        linearSlideMotor_Left = hardwareMap.dcMotor.get("linearSlideMotor_left");
        linearSlideMotor_Right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        // Reversing needed motors


        // Zero Power Behavior




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


        RightPropProcessor rightPropProcessor = new RightPropProcessor();
        rightPropProcessor.setDetectionColor(AutoFunctions.detectRed(startingPos));
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(tagProcessor,rightPropProcessor)
                .build();

        visionPortal.setProcessorEnabled(rightPropProcessor, true);

        telemetry.addData("Detect Red", AutoFunctions.detectRed(startingPos));

        waitForStart();
        runtime.reset();
        objPos = AutoFunctions.detectTeamElementright(tagProcessor,rightPropProcessor,visionPortal);
        telemetry.addData("Obj Pos", objPos);
        telemetry.addData("left perc, middle perc:", rightPropProcessor.getPercents());
        telemetry.update();

        while (opModeIsActive()) {
            if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.LEFT){
                RoadRunner.RedCloseLeftProp(telemetry, drive, linearSlideMotor_Left,linearSlideMotor_Right);
            }
            if (rightPropProcessor.getPropLocation() == CustomTypes.PropLocation.MIDDLE){
                RoadRunner.RedCloseCenterProp(telemetry, drive, linearSlideMotor_Left,linearSlideMotor_Right);
            }
            if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.RIGHT){
                RoadRunner.RedCloseRightProp(telemetry, drive, linearSlideMotor_Left,linearSlideMotor_Right);
            }
        }
    }
}
