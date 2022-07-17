package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())

                .splineToConstantHeading(new Vector2d(30, 30),  0)
                .lineToSplineHeading(new Pose2d(5, 30, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(30, 5, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        drive.followTrajectorySequence(traj);

    }
}
