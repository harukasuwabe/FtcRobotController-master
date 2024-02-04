package org.firstinspires.ftc.teamcode.Autonomous.Functions;

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
public class rightside {
    public static void leftProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
// in this code you should assume that the prop is on the left.
        Movement.backward(70,telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.backward(15, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(60,telemetry, back_left, back_right, front_left, front_right);


    }
    public static void centerProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        //places pixel to the center and oes to the april tag position
        Movement.backward(75,telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(63,telemetry, back_left, back_right, front_left, front_right);

    }
    public static void rightProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        //places pixel to the right and goes to the april tag position
        Movement.backward(70,telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(12, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.left(10,telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(65,telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);

    }//scan code thingy

}
