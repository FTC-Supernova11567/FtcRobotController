package org.firstinspires.ftc.teamcode;

/**
 * This Class is for all the constants of the robot, for example RightServoOpenAngle
 */
public class RobotConstants {
    //Gripper Class Constants
    public final static double RIGHTOPENANGLE = 0.9;
    public final static double LEFTOPENANGLE = 0;
    public final static double RIGHTCLOSEANGLE = 0.6;
    public final static double LEFTCLOSEANGLE = 0.4;

    //Arm Class Constants
    public static final int[] ticksarray = {0 ,-565 , -765, -920}   ;
    public static final double kp = 0.0035;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kf = 0;

    //TeleOp Constants
    public final static int OverrideTicksPerClick = 20;
    public final static double minSpeed = 1;
    public final static double maxSpeed = 2; // The speed the robot is at while LT is pressed
}
