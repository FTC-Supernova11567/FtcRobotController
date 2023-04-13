package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Class for all of the arms commands
 * this class uses pid Note that the 2 motors in this class are on the same Axle so they have the same encoder ticks
 * @see Gripper
 * @see Drive
 */
public class ConeThrowing {
    private DcMotorEx throwMotor;

    private RobotConstants robotConstants;

    public int set_point;
    public double motorPower;
    private int zero_position = 0;

    //PID Var
    PIDFController controller = new PIDFController(robotConstants.kp, robotConstants.ki, robotConstants.kd, robotConstants.kf);


    /**
     * Sets The zero Position of the motor.
     * Needs to be called during Init() function
     */
    public void setZeroPosition() {
        this.zero_position = throwMotor.getCurrentPosition();
    }

    /**
     * Function to get current motor position
     * @return Arm position in ticks relative to the motor zeroPosition
     */
    public int getArmPosition(){
        return throwMotor.getCurrentPosition() - this.zero_position;
    }

    public void setArmPower(double val){
        motorPower = val;
    }

    public ConeThrowing(DcMotor leftArmMotor) {
        this.throwMotor = (DcMotorEx) leftArmMotor;

        this.throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setZeroPosition();
    }

    /**
     * Function to set the arm position to the ground pole.
     * this function passes through the setposition function and uses the zeroPosition defined by the setzeroPosition function
     */
    public void ground() {
        setSet_point(zero_position);
    }

    /**
     * Function to set the arm position to the bottom pole.
     * this function passes through the setposition function
     */
    public void bottom() {
        setSet_point(robotConstants.ticksarray[1]);
    }

    /**
     * Function to set the arm position to the middle pole.
     * this function passes through the setposition function
     */
    public void middle() {
        setSet_point(robotConstants.ticksarray[2]);
    }

    /**
     * Function to set the arm position to the top pole.
     * this function passes through the setposition function
     */
    public void top() {
        setSet_point(robotConstants.ticksarray[3]);
    }

    /**
     * Function to set the position of the arm in ticks
     * @param val Value to set the arm position to it(Measured in motor ticks)
     */
    public void setSet_point(int val){
        set_point = val;
    }

    /**
     * Update Arm function Needs to be called in the Loop Function. This Function uses PIDF
     * and is setting the arm position according to the setposition function.
     */
    public void update(){
//            if (Math.abs(set_point-this.getArmPosition()) < 5 && set_point == zero_position){
//                throwMotor.setPower(0);
//            }
//            else {
//                double power = controller.calculate(this.getArmPosition(), set_point);
//                throwMotor.setPower(power);
//            }
        if (getArmPosition() >= set_point + 150){
            throwMotor.setPower(motorPower);
        }
        else {
            throwMotor.setPower(0);
        }
    }
}
