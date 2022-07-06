package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.CRServoProfiler;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class SwerveModule {
    public static PIDCoefficients MODULE_PID = new PIDCoefficients(0.5, 0, 0.05);

    public static CRServoProfiler.Constraints SERVO_CONSTRAINTS = new CRServoProfiler.Constraints(1, 1000, 2);

    public static double MAX_SERVO = 1;

    //EXPERIMENTAL FEATURES
    public static boolean WAIT_FOR_TARGET = true;

    public static double ALLOWED_ERROR = Math.toRadians(10);

    public static boolean MOTOR_FLIPPING = true;


    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDFController rotationController;
    private CRServoProfiler rotationProfiler;

    private boolean wheelFlipped = false;
    private double lastWheel = 0, currentWheel = 0;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(1000, 2000));
        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = e;
        e.setInverted(Encoder.Direction.REVERSE);

        rotationController = new PIDFController(MODULE_PID);
        rotationProfiler = new CRServoProfiler(servo, SERVO_CONSTRAINTS);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, eName)));
    }

    public void update() {
        if(MOTOR_FLIPPING){
            double delta = motor.getCurrentPosition()-lastWheel;
            currentWheel += (wheelFlipped ? -1 : 1)*delta;
            lastWheel = currentWheel;
        }
        double target = getTargetRotation(), current = getModuleRotation();
        if (current - target > Math.PI) current -= (2 * Math.PI);
        else if (target - current > Math.PI) current += (2 * Math.PI);
        servo.setPower(Range.clip(rotationController.update(current), -MAX_SERVO, MAX_SERVO));
    }

    public double getTargetRotation() {
        return rotationController.getTargetPosition();
    }

    public double getModuleRotation() {
        return encoder.getCurrentPosition();
    }

    public double getWheelPosition() {
        return DriveConstants.encoderTicksToInches(MOTOR_FLIPPING ? currentWheel : motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return DriveConstants.encoderTicksToInches(motor.getVelocity());
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(runMode, coefficients);
    }

    public void setMotorPower(double power) {
        //target check
        if(WAIT_FOR_TARGET && !isWithinAllowedError()) power = 0;
        //flip check
        else if(MOTOR_FLIPPING) power*=(wheelFlipped ? -1 : 1);
        motor.setPower(power);
    }

    public boolean isWithinAllowedError(){
        double error = Math.abs(rotationController.getLastError());
        return error < ALLOWED_ERROR || error > 2*Math.PI - ALLOWED_ERROR;
    }

    public void setServoPower(double power) {
        servo.setPower(power);
    }

    public void setTargetRotation(double target) {
        if (MOTOR_FLIPPING) {
            double current = getModuleRotation();
            //normalize for wraparound
            if (current - target > Math.PI) current -= (2 * Math.PI);
            else if (target - current > Math.PI) current += (2 * Math.PI);

            //flip target
            wheelFlipped = Math.abs(current - target) > Math.PI / 2;
            if (wheelFlipped) target = Angle.norm(target + Math.PI / 2);
        }
        rotationController.setTargetPosition(target);
    }
}
