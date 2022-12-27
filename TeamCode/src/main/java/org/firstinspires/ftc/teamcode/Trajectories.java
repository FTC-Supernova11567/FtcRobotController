package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Trajectories {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo savtashaha;
    private Servo sabashha;
    private Gripper emashha;
    public SampleMecanumDrive drive;
    private Arm abashha;
    private Trajectory bluebottomleft = null;
    private Trajectory redlefttop = null;
    private Trajectory redbottomright = null;
    private Trajectory bluetopright = null;

    public Trajectories(Servo savtashaha, Servo sabashha, Gripper emashha, SampleMecanumDrive drive, Arm abashha) {
        this.savtashaha = savtashaha;
        this.sabashha = sabashha;
        this.emashha = emashha;
        this.drive = drive;
        this.abashha = abashha;
    }
    public void setup() {
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
