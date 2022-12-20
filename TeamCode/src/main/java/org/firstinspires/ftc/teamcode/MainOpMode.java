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

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name = "mainOpMode", group = "Iterative OpMode")
public class MainOpMode extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftTop = null;
    private Motor rightTop = null;
    private Motor leftBottom = null;
    private Motor rightBottom = null;
    private DcMotor arm_Motor = null;
    private Servo gripper_servo = null;
    private MecanumDrive mecanum = null;
    private RevIMU imu;

    private Arm arm = null;
    private Gripper gripper = null;

    private double speedMultiplayer = 2;
    private final double minSpeed = 0.5;// The speed the robot is at while LT is pressed (in 1-0)
    private final double maxSpeed = 1;
    private boolean isgripperopen;

    @Override
    public void init()  {//Code to run Once
        telemetry.addData("Status", "Initialized");

        //Intialize Motors & Servos
        leftTop = new Motor(hardwareMap, "leftTop");
        rightTop = new Motor(hardwareMap, "rightTop");
        leftBottom = new Motor(hardwareMap, "leftBottom");
        rightBottom = new Motor(hardwareMap, "rightBottom");
        arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");
        gripper_servo = hardwareMap.get(Servo.class, "GripperServo");

        //MecanumDrive and imu(Gyro) Initialization
        mecanum = new MecanumDrive(leftTop, rightTop, leftBottom, rightBottom);
        imu = new RevIMU(hardwareMap);
        imu.init();

        // Tell the driver that initialization is complete.
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

//      Mecanum Drive
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;
        mecanum.driveFieldCentric(-horizontal, -vertical, -pivot, imu.getAbsoluteHeading());

//      Arm & Gripper
        arm.update();
        if (gamepad1.y){
            arm.top();
        }
        if (gamepad1.a){
            arm.bottom();
        }

        //Gripper
        if (gamepad1.b){
            if (isgripperopen){
                gripper.Close();
                isgripperopen = false;
            }
            else if (!isgripperopen){
                gripper.Open();
                isgripperopen = true;
            }
        }

//      Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.addData("Status", "Heading: " + imu.getRotation2d().getDegrees());
    }


//     Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}
