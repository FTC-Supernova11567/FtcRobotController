package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo servo;
    private Servo servo2;
    private final double RIGHTOPENANGLE = 0.9;
    private final double LEFTOPENANGLE = 0;
    private final double RIGHTCLOSEANGLE = 0.5;
    private final double LEFTCLOSEANGLE = 0.5;

    public Gripper(Servo servo, Servo servo2) {
        this.servo = servo;
        this.servo2 = servo2;
        servo2.setDirection(Servo.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void Open() {
        servo.setPosition(RIGHTOPENANGLE);
        servo2.setPosition(LEFTOPENANGLE);
    }

    public void Close() {
        servo.setPosition(RIGHTCLOSEANGLE);
        servo2.setPosition(LEFTCLOSEANGLE);
    }
}
