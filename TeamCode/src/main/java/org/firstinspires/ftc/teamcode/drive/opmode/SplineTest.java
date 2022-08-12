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

        drive.startIMUThread(this);
        if (isStopRequested()) return;
            TrajectorySequence traj = SampleSwerveDrive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(30, 30),  0)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        drive.followTrajectorySequence(traj);


    }
}
