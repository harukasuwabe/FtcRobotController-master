package org.firstinspires.ftc.teamcode.Autonomous.Functions.roadRunner;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    int startingPos = 1; // Each corresponds to the quadrant one is starting at
    /*
     * 1 = Blue closer to stage
     * 2 = Blue closer to parking
     * 3 = Red closer to stage
     * 4 = Red closer to parking
     * */
    int driveClawPos = 400;
    int objPos = 0;
    private DcMotor linearSlideMotor_Left;
    private DcMotor linearSlideMotor_Right;
    private Servo door;
    private Servo arm;

    BNO055IMU imu = null;
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        door = hardwareMap.get(Servo.class, "door");
        arm = hardwareMap.get(Servo.class, "arm");
        linearSlideMotor_Right = hardwareMap.dcMotor.get("linearSlideMotor_right");
        linearSlideMotor_Left = hardwareMap.dcMotor.get("linearSlideMotor_left");
        linearSlideMotor_Right.setDirection(DcMotor.Direction.REVERSE);
        door = hardwareMap.get(Servo.class, "door");
        arm = hardwareMap.get(Servo.class, "arm");
        linearSlideMotor_Right = hardwareMap.dcMotor.get("linearSlideMotor_right");
        linearSlideMotor_Left = hardwareMap.dcMotor.get("linearSlideMotor_left");
        linearSlideMotor_Right.setDirection(DcMotor.Direction.REVERSE);


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


        telemetry.addData("Detect Red", AutoFunctions.detectRed(startingPos));

        visionPortal.setProcessorEnabled(rightPropProcessor, true);

        waitForStart();
        runtime.reset();
        objPos = AutoFunctions.getPropLocationright(rightPropProcessor);
        telemetry.addData("Obj Pos", objPos);
        telemetry.addData("left perc, middle perc:", rightPropProcessor.getPercents());
        telemetry.update();

        while (opModeIsActive()) {
            if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.LEFT){
                RoadRunner.RedCloseLeftProp(telemetry, drive);
            }
            if (rightPropProcessor.getPropLocation() == CustomTypes.PropLocation.MIDDLE){
                RoadRunner.RedCloseCenterProp(telemetry, drive);
            }
            if (rightPropProcessor.getPropLocation()== CustomTypes.PropLocation.RIGHT){
                RoadRunner.RedCloseRightProp(telemetry, drive);
            }
        }
    }
}
