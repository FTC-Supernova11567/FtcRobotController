package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
<<<<<<< Updated upstream
    private final DcMotorEx arm_Motor;
    private final int[] ticksarray = {0, 1000, 2000, 3000}; //Needs configuration according to encoder ticks
    //PID Var

    PIDFController controller = new PIDFController(1, 0, 0, 0);
    private int state = 0;
=======
    private DcMotorEx leftArmMotor;
    private  DcMotorEx rightArmMotor;
    public int[] ticksarray = {0 ,98 , 130, 200}; //Needs configuration according to encoder ticks
    public int position;
    public double holdingPower;
    private int zero_position = 0;
    //PID Var

    PIDFController controller = new PIDFController(0.04, 0, 0.00008, 0);

>>>>>>> Stashed changes


    public Arm(DcMotor arm_Motor) {
        this.arm_Motor = (DcMotorEx) arm_Motor;
    }


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

    public void update() {
        arm_Motor.setTargetPosition((int) controller.calculate(arm_Motor.getCurrentPosition(), ticksarray[state]));
    }
}
