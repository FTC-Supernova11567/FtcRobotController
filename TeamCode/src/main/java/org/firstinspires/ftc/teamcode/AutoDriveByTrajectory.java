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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

@TeleOp(name = "AutoDriveByTrajectory", group = "Iterative OpMode")
@Disabled
public class AutoDriveByTrajectory extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo savtashaha = null;
    private Servo sabashha = null;
    private Gripper emashha = null;
    private SampleMecanumDrive drive = null;
    private Arm abashha = null;
    Trajectory bluebottomleft = null;
    Trajectory redlefttop = null;
    Trajectory redbottomright = null;
    Trajectory bluetopright = null;
    Trajectory bluebottombackup=null;
    Trajectory bluetopbackup=null;
    Trajectory redleftbackup=null;
    Trajectory redrightbackup=null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        savtashaha = hardwareMap.get(Servo.class, "GripperServo");
        sabashha = hardwareMap.get(Servo.class, "GripperServo2");
        emashha = new Gripper(sabashha, savtashaha);
        drive = new SampleMecanumDrive(hardwareMap);


        try {
            bluebottomleft = drive.trajectoryBuilder(new Pose2d(-37, 60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(-37, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(-60, 12), () -> {
                        //open gripper
                        emashha.Open();

                        //expand ar
                        abashha.middle();
                        waitSeconds(2);

                        //close gripper
                        emashha.Close();
                    })
                    .lineToLinearHeading(new Pose2d(-23.5, 12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(-23.5, 12), () -> {
                        abashha.middle();
                        waitSeconds(2);
                        emashha.Open();

                    })
                    .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-23.5, 12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-23.5, 12), () -> {
                        abashha.top();
                        waitSeconds(2);
                        emashha.Open();
                    })
                    .lineToLinearHeading(new Pose2d(-60, -60,Math.toRadians(-90)))
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();
        }

        try {
            redlefttop = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(37, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(60, 12), () -> {
                        //open gripper
                        emashha.Open();

                        //expand ar
                        abashha.middle();
                        waitSeconds(2);

                        //close gripper
                        emashha.Close();
                    })
                    .lineToLinearHeading(new Pose2d(23.5, 12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(23.5, 12), () -> {
                        abashha.middle();
                        waitSeconds(2);
                        emashha.Open();

                    })
                    .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(23.5, 12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(23.5, 12), () -> {
                        abashha.top();
                        waitSeconds(2);
                        emashha.Open();
                    })
                    .lineToLinearHeading(new Pose2d(60, 60,Math.toRadians(-90)))
                    .build();

            redbottomright = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(37, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(60, -12), () -> {
                        //open gripper
                        emashha.Open();

                        //expand ar
                        abashha.middle();
                        waitSeconds(2);

                        //close gripper
                        emashha.Close();
                    })
                    .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(23.5, -12), () -> {
                        abashha.middle();
                        waitSeconds(2);
                        emashha.Open();

                    })
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(23.5, -12), () -> {
                        abashha.top();
                        waitSeconds(2);
                        emashha.Open();
                    })
                    .lineToLinearHeading(new Pose2d(0, 60,Math.toRadians(-90)))
                    .build();
            bluetopright = drive.trajectoryBuilder(new Pose2d(-37, -60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(-37, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(-60, -12), () -> {
                        //open gripper
                        emashha.Open();

                        //expand ar
                        abashha.middle();
                        waitSeconds(2);

                        //close gripper
                        emashha.Close();
                    })
                    .lineToLinearHeading(new Pose2d(-23.5, -12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-23.5, -12), () -> {
                        abashha.middle();
                        waitSeconds(2);
                        emashha.Open();

                    })
                    .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-23.5, -12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(-23.5, -12), () -> {
                        abashha.top();
                        waitSeconds(2);
                        emashha.Open();
                    })
                    .lineToLinearHeading(new Pose2d(0, -60,Math.toRadians(-90)))
                    .build();
            redleftbackup = drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(-90)))
                            .lineToLinearHeading(new Pose2d(37, -37,Math.toRadians(-90)))
                            .build();
            redrightbackup = drive.trajectoryBuilder(new Pose2d(-37, 60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(-37, 37,Math.toRadians(-90)))
                    .build();
            bluetopbackup = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(37, 37,Math.toRadians(-90)))
                    .build();
            bluebottombackup = drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(-60, -37,Math.toRadians(-90)))
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();

        }

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
        drive.followTrajectory(bluebottomleft);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

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
