package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    private DcMotorEx Arm_Motor = null;
    private int[] ticksarray = {0, 1000, 2000, 3000}; //Needs configuration according to encoder ticks
    //PID Var
    double integralSum = 0;
    double Kp = 0;
    private int state = 0;


    ElapsedTime timer = new ElapsedTime();
    private  double lastError = 0;


    public Arm(DcMotor arm_Motor) {
        Arm_Motor = (DcMotorEx) arm_Motor;
        this.Arm_Motor.setPositionPIDFCoefficients(Kp);
    }


    public void update() {
        // if motor not close enough to set point then:
        if (lastError < 3) {//Configure
            Arm_Motor.setTargetPosition(ticksarray[state]);
        }
    }
    public void StateUp(){
        if (state < 3){
            state++;
        }
    }
    public void StateDown(){
        if (state > 0){
            state--;
        }
    }

    /*public double PIDController(double reference, double state){
        double error = reference - state;
        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
*/}
