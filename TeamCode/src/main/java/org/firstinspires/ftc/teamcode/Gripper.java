package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo servo;
    private Servo servo2;
    private final double OPENANGLE = 46;
    private final double CLOSEANGLE = 2;

    public Gripper(Servo servo, Servo servo2) {
        this.servo = servo;
        this.servo2 = servo2;
        servo2.setDirection(Servo.Direction.FORWARD);
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Open() {
        servo.setPosition(OPENANGLE);
        servo2.setPosition(OPENANGLE);
    }

    public void Close() {
        servo.setPosition(CLOSEANGLE);
        servo2.setPosition(CLOSEANGLE);
    }
}
