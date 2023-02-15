package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private RobotConstants robotConstants;
    private Servo servo;
    private Servo servo2;

    public Gripper(Servo servo, Servo servo2) {
        this.servo = servo;// Right Servo
        this.servo2 = servo2;// Left Servo
        servo2.setDirection(Servo.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void Open() {
        servo.setPosition(robotConstants.RIGHTOPENANGLE);
        servo2.setPosition(robotConstants.LEFTOPENANGLE);
    }

    public void Close() {
        servo.setPosition(robotConstants.RIGHTCLOSEANGLE);
        servo2.setPosition(robotConstants.LEFTCLOSEANGLE);
    }
}
