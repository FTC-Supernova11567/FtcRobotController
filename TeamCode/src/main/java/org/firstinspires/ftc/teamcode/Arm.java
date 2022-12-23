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
    //PID Var

    PIDFController controller = new PIDFController(0.05, 0, 0, 0);
    int state = 0;



    public Arm(DcMotor leftArmMotor, DcMotor rightArmMotor) {
        this.leftArmMotor = (DcMotorEx) leftArmMotor;
        this.rightArmMotor = (DcMotorEx) rightArmMotor;

        this.leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void boomBoomControl(int targetPosition){
        if (Math.abs(leftArmMotor.getCurrentPosition() - targetPosition) > TOLERANCE){
            leftArmMotor.setPower(BOOMBOOMPOWER * Math.signum(leftArmMotor.getCurrentPosition() - targetPosition));
            rightArmMotor.setPower(BOOMBOOMPOWER * Math.signum(leftArmMotor.getCurrentPosition() - targetPosition));
        }
        else {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
        }
    }

    public void setTOLERANCE(int TOLERANCE) {
        this.TOLERANCE = TOLERANCE;
    }

    public void setBOOMBOOMPOWER(double BOOMBOOMPOWER) {
        this.BOOMBOOMPOWER = BOOMBOOMPOWER;
    }

    public void setPosition(int position){
        leftArmMotor.setPower(controller.calculate(leftArmMotor.getCurrentPosition(), position));
        rightArmMotor.setPower( controller.calculate(leftArmMotor.getCurrentPosition(), position));
    }
    public void update() {
        leftArmMotor.setPower(controller.calculate(leftArmMotor.getCurrentPosition(), ticksarray[state]));
        rightArmMotor.setPower(controller.calculate(leftArmMotor.getCurrentPosition(), ticksarray[state]));
    }
}
