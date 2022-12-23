package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo servo;
    private Servo servo2;
    private final double OPENANGLE = 1;
    private final double CLOSEANGLE = 0.2;

    public Gripper(Servo servo, Servo servo2) {
        this.servo = servo;
        this.servo2 = servo2;
        servo2.setDirection(Servo.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void Open() {
        servo.setPosition(OPENANGLE);
        servo2.setPosition(OPENANGLE);
    }

    public void Close() {
        servo.setPosition(CLOSEANGLE);
        servo2.setPosition(CLOSEANGLE);
    }

    public boolean IsClose(){
        if (servo.getPosition() == CLOSEANGLE && servo2.getPosition() == CLOSEANGLE){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean IsOpen(){
        if (servo.getPosition() == OPENANGLE && servo2.getPosition() == OPENANGLE){
            return true;
        }
        else{
            return false;
        }
    }
}
