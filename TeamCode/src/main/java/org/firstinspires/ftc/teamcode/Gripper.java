package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class for all of the gripper commands
 * recommend to adjust Open & Close angle accordingly
 * @see Drive
 * @see Arm
 */
public class Gripper {
    private Servo servo;
    private Servo servo2;
    private final double RIGHTOPENANGLE = 0.9;
    private final double LEFTOPENANGLE = 0;
    private final double RIGHTCLOSEANGLE = 0.5;
    private final double LEFTCLOSEANGLE = 0.5;


    /**
     * Gripper Class Constructor
     * @param servo Right Gripper Servo
     * @param servo2 Left Gripper Servo
     * @see Arm
     * @see Drive
     */
    public Gripper(Servo servo, Servo servo2) {
        this.servo = servo;
        this.servo2 = servo2;
        servo2.setDirection(Servo.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Function to open the gripper,
     * Uses each servo open angle
     */
    public void Open() {
        servo.setPosition(RIGHTOPENANGLE);
        servo2.setPosition(LEFTOPENANGLE);
    }

    /**
     * Function to close the gripper
     * Uses each servo close angle
     */
    public void Close() {
        servo.setPosition(RIGHTCLOSEANGLE);
        servo2.setPosition(LEFTCLOSEANGLE);
    }
}
