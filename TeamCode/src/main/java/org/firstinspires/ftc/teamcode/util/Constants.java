package org.firstinspires.ftc.teamcode.util;

public class Constants {

    public static double cameraHeight = 0.0;
    public static double cameraPerpendicularOffset = -7.5; // forward is positive
    public static double cameraLateralOffset = 0.5; // to the right is positive

    public enum SlidePositions {
        DOWN(0),
        LOW(800),
        MIDDLE(1400),
        HIGH(1900),
        LIMIT(2300);

        public final int position;

        SlidePositions(int position) {this.position = position;}
        public int getPosition() {return this.position;}
    }

    public enum SlidesPIDF {
        kp(0.005),
        ki(0.0),
        kd(0.0),
        kg(0.07);

        public final double value;

        SlidesPIDF(double value) {this.value = value;}
        public double getValue() {return this.value;}
    }
}