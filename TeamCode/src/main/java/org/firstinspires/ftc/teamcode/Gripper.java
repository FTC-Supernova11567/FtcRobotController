package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo servo;
    private double OPENANGLE = 46;
    private double CLOSEANGLE = 2;

    public Gripper(Servo servo){
        this.servo = servo;
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Open() {
    servo.setPosition(OPENANGLE);
    }

    public void Close(){
        servo.setPosition(CLOSEANGLE);
    }
}
