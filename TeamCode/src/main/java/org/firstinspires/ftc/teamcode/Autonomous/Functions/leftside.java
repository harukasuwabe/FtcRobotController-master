package org.firstinspires.ftc.teamcode.Autonomous.Functions;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Movement;

public class leftside {
    public static void leftProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
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

    public static void centerProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
        Movement.backward(65, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);
        Movement.forward(55, telemetry, back_left, back_right, front_left, front_right);
        Thread.sleep(100);   // this code just drops the block and then places it and goes back to starting position


    }
    public static void rightProp(Telemetry telemetry, DcMotor back_left, DcMotor back_right, DcMotor front_left, DcMotor front_right) throws InterruptedException {
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
