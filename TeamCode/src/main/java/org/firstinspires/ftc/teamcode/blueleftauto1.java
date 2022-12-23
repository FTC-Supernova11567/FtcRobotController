package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class blueleftauto1 {

    public class bluerightauto1 {




        public class redleftauto1 {
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

            @TeleOp(name = "AutoDriveByTrajectory3", group = "Iterative OpMode")
            @Disabled
            public class AutoDriveByTrajectory3 extends OpMode {
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

                @Override
                public void init() {
                    telemetry.addData("Status", "Initialized");
                    savtashaha = hardwareMap.get(Servo.class, "GripperServo");
                    sabashha = hardwareMap.get(Servo.class, "GripperServo2");
                    emashha = new Gripper(sabashha, savtashaha);
                    drive = new SampleMecanumDrive(hardwareMap);

                    bluetopbackup = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                            .lineToLinearHeading(new Pose2d(37, 37,Math.toRadians(-90)))
                            .build();
                }
                @Override
                public void start() {
                    runtime.reset();
                    drive.followTrajectory(bluetopbackup);
                }

            }
