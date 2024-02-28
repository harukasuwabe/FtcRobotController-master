package org.firstinspires.ftc.teamcode.Autonomous.Functions;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class leftside {


    public static void leftProp_RR(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(15,-59, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory leftprop = sampleMecanumDrive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(0,-20.5), Math.toRadians(180))
                .build();

        sampleMecanumDrive.followTrajectory(leftprop);
    }
    public static void centerProp_RR(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(15,-59, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory centerprop = sampleMecanumDrive.trajectoryBuilder(startPose)
                .back(35)
                .build();

        sampleMecanumDrive.followTrajectory(centerprop);
    }
    public static void leftProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
// in this code you should assume that the prop is on the left.
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);

        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        //Movement.backward(5,telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(65, telemetry, back_left, back_right, front_left, front_right);
        Movement.right(5, telemetry, back_left, back_right, front_left, front_right);


    }


    public static void centerProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        Movement.backward(65, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(55, telemetry, back_left, back_right, front_left, front_right);


    }
    public static void rightProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        Movement.backward(70, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationRight(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.backward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(15, telemetry, back_left, back_right, front_left, front_right);
        Movement.rotationLeft(90, telemetry, back_left, back_right, front_left, front_right);
        Movement.forward(60, telemetry, back_left, back_right, front_left, front_right);

    }//scan code thingy
}
