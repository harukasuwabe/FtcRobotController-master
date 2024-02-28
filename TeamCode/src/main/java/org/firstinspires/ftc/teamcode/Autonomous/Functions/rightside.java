package org.firstinspires.ftc.teamcode.Autonomous.Functions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
public class rightside {


    public static void rightProp_RR(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(15,59, Math.toRadians((90)));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory rightprop = sampleMecanumDrive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(24,23.5), Math.toRadians(270))
                .build();

        sampleMecanumDrive.followTrajectory(rightprop);
    }
    public static void leftProp_RR(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(15,59, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory leftprop = sampleMecanumDrive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0,23.5), Math.toRadians(0))
                .build();

        sampleMecanumDrive.followTrajectory(leftprop);
    }
    public static void centerProp_RR(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(35,59, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory centerprop = sampleMecanumDrive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(12,24), Math.toRadians(90))
                .build();

        sampleMecanumDrive.followTrajectory(centerprop);
    }
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
