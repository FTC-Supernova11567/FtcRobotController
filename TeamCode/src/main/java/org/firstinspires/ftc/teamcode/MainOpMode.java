/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@TeleOp(name = "mainOpMode", group = "Iterative OpMode")
public class MainOpMode extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop = null;
    private DcMotor rightTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightBottom = null;
    private DcMotor right_arm_motor = null;
    private DcMotor left_arm_motor = null;
    private Servo rightServo = null;
    private Servo leftServo = null;
    private Gripper gripper = null;
    private Drive drive = null;
    private Arm arm = null;
    private RevIMU imu;


    private final double minSpeed = 1;
    private final double maxSpeed = 2;// The speed the robot is at while LT is pressed
    private final double overridePower = 0.5;

    private double speedMultiplayer = 2;

    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        left_arm_motor = hardwareMap.get(DcMotor.class, "LeftArmMotor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "RightArmMotor");

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new Drive(leftTop, rightTop, leftBottom, rightBottom);
        gripper = new Gripper(rightServo, leftServo);
        arm = new Arm(left_arm_motor, right_arm_motor);

        leftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        right_arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        arm.setZeroPosition();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.2) {
            speedMultiplayer = minSpeed;
        } else {
            speedMultiplayer = maxSpeed;
        }

        if (gamepad2.right_trigger > 0.2) {
            gripper.Open();
        }
        if (gamepad2.left_trigger > 0.2) {
            gripper.Close();
        }

        if (gamepad2.a){
            arm.ground();
        }
        else if (gamepad2.b){
            arm.bottom();
        }
        else if (gamepad2.y){
            arm.middle();
        }
        else if (gamepad2.x){
            arm.top();
        }

//        else if (gamepad2.dpad_up){
//            right_arm_motor.setPower(overridePower);
//            left_arm_motor.setPower(overridePower);
//        }
//
//        else if (gamepad2.dpad_down){
//            right_arm_motor.setPower(-overridePower);
//            left_arm_motor.setPower(-overridePower);
//        }

        arm.update();
        drive.go(gamepad1.left_stick_x, -gamepad1.left_stick_y, deadzone(gamepad1.right_stick_x), speedMultiplayer, -imu.getRotation2d().getRadians());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Degrees", leftServo.getPosition());
        telemetry.addData("Status", "Heading: " + imu.getRotation2d().getDegrees());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public double deadzone(double val) {
        if (val < 0.05 && val > -0.05) {
            return 0;
        }
        return val;
    }

}
