package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    private DcMotor Arm_Motor = null;

    //PID Var
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private  double lastError = 0;

    //Number of ticks to the pillars
    private final int Pillar1 = 1000;
    private final int Pillar2 = 2000;
    private final int Pillar3 = 3000;

    public Arm(DcMotor arm_Motor) {
        Arm_Motor = arm_Motor;
    }



    public void Cone1(){
        Arm_Motor.setPower(PIDController(Pillar1, Arm_Motor.getCurrentPosition()));
    }
    public void Cone2(){
        Arm_Motor.setPower(PIDController(Pillar2, Arm_Motor.getCurrentPosition()));
    }
    public void Cone3(){
        Arm_Motor.setPower(PIDController(Pillar3, Arm_Motor.getCurrentPosition()));
    }




    private double PIDController(double reference, double state){
        double error = reference - state;
        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
}
