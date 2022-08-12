package org.firstinspires.ftc.teamcode.drive.opmode;


import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ConfigurablePose;

import java.util.ArrayList;
import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class AutoPathTest extends LinearOpMode {
    public static double CYCLES = 8;
    public static ConfigurablePose CYCLE_START = new ConfigurablePose(12, -63, toRadians(90));
    public static ConfigurablePose ALLIANCE_HUB = new ConfigurablePose(8, -36, toRadians(180));
    public static double HUB_APPROACH_HEADING = Math.toRadians(90);
    public static double HUB_LEAVE_HEADING = Math.toRadians(-90);
    public static ConfigurablePose CYCLE_TRENCH = new ConfigurablePose(24, -64, toRadians(180));
    public static double TRENCH_APPROACH_HEADING = Math.toRadians(0);
    public static ConfigurablePose[] AUTO_WAREHOUSE = new ConfigurablePose[]{
            new ConfigurablePose(44, -64, toRadians(180)),
            new ConfigurablePose(46, -64, toRadians(180)),
            new ConfigurablePose(48, -64, toRadians(180)),
            new ConfigurablePose(50, -64, toRadians(180)),
            new ConfigurablePose(51, -64, toRadians(180)),
            new ConfigurablePose(52, -64, toRadians(180)),
            new ConfigurablePose(53, -64, toRadians(180)),
            new ConfigurablePose(53, -64, toRadians(180)),
            new ConfigurablePose(53, -64, toRadians(180)),

    };

    @Override
    public void runOpMode() throws InterruptedException {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);
        drive.setPoseEstimate(CYCLE_START.toPose());
        drive.setExternalHeading(CYCLE_START.getHeading());

        List<TrajectorySequence> sequences = new ArrayList<>();
        sequences.add(SampleSwerveDrive.trajectorySequenceBuilder(CYCLE_START.toPose())
                .lineToLinearHeading(ALLIANCE_HUB.toPose())
                .build());
        for (int j = 0; j < CYCLES; j++) {
            sequences.add(SampleSwerveDrive.trajectorySequenceBuilder(ALLIANCE_HUB.toPose(), HUB_LEAVE_HEADING)
                    .splineToConstantHeading(CYCLE_TRENCH.toVec(), TRENCH_APPROACH_HEADING)
                    .lineToConstantHeading(AUTO_WAREHOUSE[j].toVec()).build());
            sequences.add(SampleSwerveDrive.trajectorySequenceBuilder(AUTO_WAREHOUSE[j].toPose())
                    .lineToConstantHeading(CYCLE_TRENCH.toVec())
                    .splineToConstantHeading(ALLIANCE_HUB.toVec(), HUB_APPROACH_HEADING)
                    .build());
        }
        sequences.add(SampleSwerveDrive.trajectorySequenceBuilder(ALLIANCE_HUB.toPose(), HUB_LEAVE_HEADING)
                .splineToConstantHeading(CYCLE_TRENCH.toVec(), TRENCH_APPROACH_HEADING)
                .lineToConstantHeading(AUTO_WAREHOUSE[0].toVec()).build());

        waitForStart();


        drive.startIMUThread(this);
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()){
            if(!drive.isBusy()){
                if(sequences.isEmpty()) break;
                drive.followTrajectorySequenceAsync(sequences.remove(0));
            }
            drive.update();

        }


    }
}
