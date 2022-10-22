package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class LongerStraightTest extends LinearOpMode {
    public static double DISTANCE = 30; // in
    int loopCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(DISTANCE)
                .back(DISTANCE)
                .build();

        waitForStart();
        drive.startIMUThread(this);
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            drive.followTrajectorySequence(trajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("Loop", loopCount);
            telemetry.update();
            loopCount++;
        }
    }
}
