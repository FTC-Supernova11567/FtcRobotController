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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

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

@TeleOp(name = "LeftTrajectoryTesting", group = "")
public class AutonomousLeft extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor right_arm_motor = null;
    private DcMotor left_arm_motor = null;
    private Servo rightServo = null;
    private Servo leftServo = null;

    private Gripper gripper = null;
    private Arm arm = null;
    private SampleMecanumDrive drive = null;
    private AprilTagDetector aprilTagDetector = null;

    private int id;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        drive = new SampleMecanumDrive(hardwareMap);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetector = new AprilTagDetector(camera, hardwareMap, telemetry);
//        aprilTagDetector.init();
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        left_arm_motor = hardwareMap.get(DcMotor.class, "LeftArmMotor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "RightArmMotor");

        right_arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper = new Gripper(rightServo, leftServo);
        arm = new Arm(left_arm_motor, right_arm_motor);

        arm.setZeroPosition();
//        arm.controller.setTolerance(7);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
//        aprilTagDetector.detectTag();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        gripper.Close();
        waitSeconds(0.5);
        runtime.reset();
//        id = aprilTagDetector.publishResult();
//        switch (id){
//            case 1:
//                drive.followTrajectory(trajectories.barcodCase1);
//                break;
//            case 2:
//                drive.followTrajectory(trajectories.barcodCase2);
//                break;
//            case 3:
//                drive.followTrajectory(trajectories.barcodCase3);
//                break;
//        }
        TrajectorySequence blueLeft = drive.trajectorySequenceBuilder(new Pose2d(-35, -58, Math.toRadians(90)))
                .setVelConstraint(new TranslationalVelocityConstraint(15))
                .forward(46)
                .waitSeconds(2)
                .addDisplacementMarker(() ->{
                    arm.top();
                })
                .waitSeconds(3)
                .turn(Math.toRadians(-45))
                .addTemporalMarker(17, () -> {
                    gripper.Open();
                })
                .build();
        drive.followTrajectorySequenceAsync(blueLeft);
    }

    @Override
    public void loop() {
        drive.update();
        arm.update();
        telemetry.addData("arm_position", left_arm_motor.getPower());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void waitSeconds(double seconds) {
        try {
            Thread.sleep((long) (1000 * seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
