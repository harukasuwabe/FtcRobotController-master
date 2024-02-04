package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.HSV.PropThreshold;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class leftside {
    public static void leftProp(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
// in this code you should assume that the prop is on the left.
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);

        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        //Movement.backward(5,telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(65, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);


    }

    public static void centerProp(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        Movement.backward(65, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(55, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);   // this code just drops the block and then places it and goes back to starting position


    }
    public static void rightProp(DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.backward(15, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(60, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);

    }//scan code thingy
}
