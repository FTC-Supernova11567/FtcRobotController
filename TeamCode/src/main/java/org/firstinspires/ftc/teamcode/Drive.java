package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.widget.HorizontalScrollView;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop = null;
    private DcMotor rightTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightBottom = null;

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
    }

    public void go(double horizontal, double vertical, double pivot){
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vertical, horizontal, pivot));

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftTop.setPower(wheelSpeeds.frontLeftMetersPerSecond);
        rightTop.setPower(wheelSpeeds.frontRightMetersPerSecond);
        leftBottom.setPower(wheelSpeeds.rearLeftMetersPerSecond);
        rightBottom.setPower(wheelSpeeds.rearRightMetersPerSecond);

        // Show the elapsed game time and wheel power.

    }



    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


}
