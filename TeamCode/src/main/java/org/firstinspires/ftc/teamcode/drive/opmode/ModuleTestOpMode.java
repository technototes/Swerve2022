package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.CRServoProfiler;

import java.util.List;
@Disabled
@Config
@TeleOp(name="module test")
public class ModuleTestOpMode extends LinearOpMode {
    public static double POS_ADJ= 2;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, "leftRearMotor");
        PIDFCoefficients coefficients = DriveConstants.MOTOR_VELO_PID;
        m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
        CRServo s = hardwareMap.get(CRServo.class, "leftRearServo");
        CRServoProfiler profiler = new CRServoProfiler(s, SampleSwerveDrive.SERVO_CONSTRAINTS);
        PIDFController rotationController = new PIDFController(SampleSwerveDrive.MODULE_PID);
        double servoPosition = 0;
        ElapsedTime loop = new ElapsedTime();
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double currentRot = 0, currentVel = 0;
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("vel: "+currentVel);
            telemetry.addLine("rot: "+currentRot);




            double deltaSec = loop.seconds();
            loop.reset();

            if(gamepad1.dpad_up) currentVel+=1*deltaSec;
            if(gamepad1.dpad_down) currentVel-=1*deltaSec;
            if(gamepad1.dpad_left && Math.abs(rotationController.getLastError()) < 0.01) currentRot-=0.5;
            if(gamepad1.dpad_right&& Math.abs(rotationController.getLastError()) < 0.01) currentRot+=0.5;

            if(gamepad1.circle) currentVel = 0;
            if(gamepad1.b) currentRot = 0;

            telemetry.update();
            rotationController.setTargetPosition(currentRot);
            m.setPower(currentVel);
            servoPosition+=POS_ADJ*deltaSec*s.getPower();
            profiler.setTarget(rotationController.update(servoPosition));
            for (LynxModule module : modules) {
                module.clearBulkCache();
            }
        }
    }
}
