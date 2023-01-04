package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class Arm {
    private DcMotorEx leftArmMotor;
    private  DcMotorEx rightArmMotor;
    private int TOLERANCE = 0;
    private double BOOMBOOMPOWER = 0.1;
    public int[] ticksarray = {0, -20, -50, -100}; //Needs configuration according to encoder ticks
    private int zero_position = 0;
    //PID Var

    PIDFController controller = new PIDFController(0.05, 0, 0, 0);

    int state = 0;

    public void setZero_position() {
        this.zero_position = leftArmMotor.getCurrentPosition();
    }

    public int getArmPosition(){
        return leftArmMotor.getCurrentPosition();
    }

    public Arm(DcMotor leftArmMotor, DcMotor rightArmMotor) {
        this.leftArmMotor = (DcMotorEx) leftArmMotor;
        this.rightArmMotor = (DcMotorEx) rightArmMotor;

        this.leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setZero_position();
    }

    // Autonomous Functions
    public void idle() {
        state = 0;
    }

    public void bottom() {
        state = 1;
    }

    public void middle() {
        state = 2;
    }

    public void top() {
        state = 3;
    }

    //Non Autonomous Functions
    public void StateUp(){
        state++;
    }

    public void StateDown(){
        state--;
    }

    public void setPosition(int position){
        leftArmMotor.setPower(controller.calculate(this.getArmPosition(), position));
        rightArmMotor.setPower( controller.calculate(this.getArmPosition(), position));
    }
    public void update() {
        double power = controller.calculate(leftArmMotor.getCurrentPosition(), zero_position + ticksarray[state]);
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }
}
