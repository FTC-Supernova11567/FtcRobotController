package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Trajectories {
    private final ElapsedTime runtime = new ElapsedTime();
    private Gripper gripper;
    public SampleMecanumDrive drive;
    public Arm arm;
    public Trajectory bluebottomleft = null;
    public Trajectory redlefttop = null;
    public Trajectory redbottomright = null;
    public Trajectory bluetopright = null;
    public Trajectory barcodCase1 = null;
    public Trajectory barcodCase2 = null;
    public Trajectory barcodCase3 = null;

    public Trajectories(Gripper gripper, SampleMecanumDrive drive, Arm arm) {
        this.gripper = gripper;
        this.drive = drive;
        this.arm = arm;
    }

    public void setup() {
        try {
            bluebottomleft = drive.trajectoryBuilder(new Pose2d(-37, 60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(-37, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(-60, 12), () -> {
                        //open gripper
                        gripper.Open();

                        //expand ar
                        arm.middle();
                        waitSeconds(2);

                        //close gripper
                        gripper.Close();
                    })
                    .lineToLinearHeading(new Pose2d(-23.5, 12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(-23.5, 12), () -> {
                        arm.middle();
                        waitSeconds(2);
                        gripper.Open();

                    })
                    .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-23.5, 12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-23.5, 12), () -> {
                        arm.top();
                        waitSeconds(2);
                        gripper.Open();
                    })
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
                        gripper.Open();

                        //expand ar
                        arm.middle();
                        waitSeconds(2);

                        //close gripper
                        gripper.Close();
                    })
                    .lineToLinearHeading(new Pose2d(23.5, 12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(23.5, 12), () -> {
                        arm.middle();
                        waitSeconds(2);
                        gripper.Open();

                    })
                    .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(23.5, 12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(23.5, 12), () -> {
                        arm.top();
                        waitSeconds(2);
                        gripper.Open();
                    })
                    .build();

            redbottomright = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(37, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(60, -12), () -> {
                        //open gripper
                        gripper.Open();

                        //expand ar
                        arm.middle();
                        waitSeconds(2);

                        //close gripper
                        gripper.Close();
                    })
                    .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(23.5, -12), () -> {
                        arm.middle();
                        waitSeconds(2);
                        gripper.Open();

                    })
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(23.5, -12), () -> {
                        arm.top();
                        waitSeconds(2);
                        gripper.Open();
                    })
                    .build();
            bluetopright = drive.trajectoryBuilder(new Pose2d(-37, -60, Math.toRadians(-90)))
                    //.strafeR
                    .lineToLinearHeading(new Pose2d(-37, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(-60, -12), () -> {
                        //open gripper
                        gripper.Open();

                        //expand ar
                        arm.middle();
                        waitSeconds(2);

                        //close gripper
                        gripper.Close();
                    })
                    .lineToLinearHeading(new Pose2d(-23.5, -12, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-23.5, -12), () -> {
                        arm.middle();
                        waitSeconds(2);
                        gripper.Open();

                    })
                    .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-23.5, -12, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(-23.5, -12), () -> {
                        arm.top();
                        waitSeconds(2);
                        gripper.Open();
                    })
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();
        }
        try {
            barcodCase1 = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(37, 14, Math.toRadians(-90)))
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();
        }
        try {
            barcodCase2 = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(37, 14, Math.toRadians(-90)))
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();
        }
        try {
            barcodCase3 = drive.trajectoryBuilder(new Pose2d(37, 60, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(37, 14, Math.toRadians(-90)))
                    .build();
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }
    public void waitSeconds ( double seconds){
        try {
            Thread.sleep((long) (1000 * seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
