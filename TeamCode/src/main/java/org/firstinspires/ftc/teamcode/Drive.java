package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Drive Class
 * class for all of the drive commands
 * this class is for mecanum wheels
 * @see Gripper
 * @see Arm
 */
public class Drive {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop = null;
    private DcMotor rightTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightBottom = null;


    /**
     * Drive class constructor
     * @param leftTop leftTop DCMotor
     * @param rightTop rightTop DCMotor
     * @param leftBottom leftBottom DCMotor
     * @param rightBottom rightBottom DCMotor
     */
    public Drive(DcMotor leftTop, DcMotor rightTop, DcMotor leftBottom, DcMotor rightBottom) {
        this.leftTop = leftTop;
        this.rightTop = rightTop;
        this.leftBottom = leftBottom;
        this.rightBottom = rightBottom;
    }


    /**
     * Funtion to Calculate the required motor power of all motors and send it to them
     * This Function needs to be called in the Loop section of the OpMode.
     * Bot heading is measured in Radians (-imu.getRotation2d().getRadians())
     * @param x Controller right joystick X
     * @param y Controller left joystick Y
     * @param rx Controller right joystick X
     * @param speedmultiplayer Drive Speed Multiplayer
     * @param botHeading Robot Heading (-imu.getRotation2d().getRadians())
     */
    public void go(double x, double y, double rx, double speedmultiplayer, double botHeading) {
        // Read inverse IMU heading, as the IMU heading is CW positive


        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * speedmultiplayer;
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftTop.setPower(frontLeftPower);
        leftBottom.setPower(backLeftPower);
        rightTop.setPower(frontRightPower);
        rightBottom.setPower(backRightPower);
    }
}
