package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "PIDTuner", group = "Iterative OpMode")
public class PIDTuner extends LinearOpMode {
    public static double kP = 1.5;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double velocitySetPoint = 0;
    public static int positionSetPoint = 0;
    public static boolean isVelocityMode = true;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        DcMotor motor =  hardwareMap.get(DcMotor.class, "TestMotor");
//        motor.setVelocity(0);
//        motor.setVelocityPIDFCoefficients(kP,kI,kD,kF);
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            motor.setPower(kP);

//            telemetry.addData("Motor Velocity", motor.getVelocity()
//            );
//            telemetry.addData("Motor Position", motor.getCurrentPosition());
//            telemetry.update();

            sleep(20);
        }
    }
}
