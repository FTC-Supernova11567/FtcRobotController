package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    private DcMotorEx leftArmMotor;
    private  DcMotorEx rightArmMotor;
    public int[] ticksarray = {0 ,85 , 130, 170}; //Needs configuration according to encoder ticks
    public int position;
    public double holdingPower;
    private int zero_position = 0;
    //PID Var

    PIDFController controller = new PIDFController(0.068, 0, 0.0035, 0);

//
// @params:
//   @return:
    public void setZeroPosition() {
        this.zero_position = leftArmMotor.getCurrentPosition();
    }

    public void setHoldingPower(double val) {
        val = holdingPower;
    }

    public int getArmPosition(){
        return leftArmMotor.getCurrentPosition() - this.zero_position;
    }

    public Arm(DcMotor leftArmMotor, DcMotor rightArmMotor) {
        this.leftArmMotor = (DcMotorEx) leftArmMotor;
        this.rightArmMotor = (DcMotorEx) rightArmMotor;

        this.leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setZeroPosition();
        controller.setTolerance(10);
    }

    // Autonomous Functions
    public void ground() {
        setPosition(zero_position);
    }

    public void bottom() {
        setPosition(ticksarray[1]);
    }

    public void middle() {
        setPosition(ticksarray[2]);
    }

    public void top() {
        setPosition(ticksarray[3]);
    }

    public void setPosition(int val){
        position = val;
    }

    public void update(){
            if (position == zero_position){
                rightArmMotor.setPower(0);
                leftArmMotor.setPower(0);
            }
            else {
                double power = controller.calculate(this.getArmPosition(), position);
                leftArmMotor.setPower(power);
                rightArmMotor.setPower(power);
            }
    }
}
