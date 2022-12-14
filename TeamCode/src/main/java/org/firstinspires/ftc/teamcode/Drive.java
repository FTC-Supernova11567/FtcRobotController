package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop = null;
    private DcMotor rightTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightBottom = null;
    private double Kp = 0;//Needs to be Configurable
    private double Ki = 0;
    private double Kd = 0;

    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(11, 14.25),
            new Translation2d(11, -14.25),
            new Translation2d(-11, 14.25),
            new Translation2d(-11, -14.25)


    );

    public Drive(DcMotor leftTop, DcMotor rightTop, DcMotor leftBottom, DcMotor rightBottom) {
        this.leftTop = leftTop;
        this.rightTop = rightTop;
        this.leftBottom = leftBottom;
        this.rightBottom = rightBottom;
//        this.leftTop.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
    }

    public void go(double horizontal, double vertical, double pivot) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vertical, horizontal, pivot));

        // Send calculated power to wheels
        leftTop.setPower(wheelSpeeds.frontLeftMetersPerSecond /  6000/40);
        rightTop.setPower(wheelSpeeds.frontRightMetersPerSecond / 6000/40);
        leftBottom.setPower(wheelSpeeds.rearLeftMetersPerSecond / 6000/40);
        rightBottom.setPower(wheelSpeeds.rearRightMetersPerSecond / 6000/40);


    }

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
}
