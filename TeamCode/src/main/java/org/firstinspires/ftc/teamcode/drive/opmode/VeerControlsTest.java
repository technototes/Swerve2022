package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class VeerControlsTest extends LinearOpMode {
    public static PIDCoefficients HEADING_COEFFS = new PIDCoefficients(0.5, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime t = new ElapsedTime();
        PIDFController headingController = new PIDFController(HEADING_COEFFS);
        waitForStart();


        drive.startIMUThread(this);

        double target = 0;

        while (!isStopRequested()) {
            double current = drive.getExternalHeading();
            if(Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2){
                target = Angle.norm(-Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)-Math.PI/2);
                headingController.setTargetPosition(target);
            }
            if (current - target > Math.PI) current -= (2 * Math.PI);
            else if (target - current > Math.PI) current += (2 * Math.PI);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                                    ).rotated(-current),
                            headingController.update(current)
                    )
            );
            if(gamepad1.right_stick_button) drive.setExternalHeading(0);

            drive.updateModules();

            telemetry.addData("looptime",1/t.seconds());
            t.reset();

            telemetry.update();
        }
    }
}
