package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    public static PIDCoefficients MODULE_PID = new PIDCoefficients(0.4, 0, 0.05);

    public static double FEEDFORWARD = 0.1;

    public static CRServoProfiler.Constraints SERVO_CONSTRAINTS = new CRServoProfiler.Constraints(1, 1000, 2);

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    //EXPERIMENTAL FEATURES
    public static boolean WAIT_FOR_TARGET = false;

    public static double ALLOWED_COS_ERROR = Math.toRadians(2);

    public static double ALLOWED_SERVO_STOP_ERROR = Math.toRadians(2);

    public static boolean MOTOR_FLIPPING = true;

    public static double FLIP_BIAS = Math.toRadians(0);


    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDFController rotationController;
    private CRServoProfiler rotationProfiler;

    private boolean wheelFlipped = false;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(1000, 2000));
        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = e;
        e.setInverted(Encoder.Direction.REVERSE);

        rotationController = new PIDFController(MODULE_PID, 0, 0, FEEDFORWARD);
        rotationProfiler = new CRServoProfiler(servo, SERVO_CONSTRAINTS);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, eName)));
    }

    public void update() {
        double target = getTargetRotation(), current = getModuleRotation();
        if (current - target > Math.PI) current -= (2 * Math.PI);
        else if (target - current > Math.PI) current += (2 * Math.PI);
        double power = Range.clip(rotationController.update(current), -MAX_SERVO, MAX_SERVO);
        servo.setPower(Math.abs(rotationController.getLastError()) > ALLOWED_SERVO_STOP_ERROR ? power : 0);
    }

    public double getTargetRotation() {
        return rotationController.getTargetPosition();
    }

    public double getModuleRotation() {
        return encoder.getCurrentPosition();
    }

    public double getWheelPosition() {
        return DriveConstants.encoderTicksToInches(motor.getCurrentPosition());
    }
    public int flipModifier(){
        return MOTOR_FLIPPING && wheelFlipped ? 1 : -1;
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
        if(WAIT_FOR_TARGET && !isWithinAllowedError()){
            power*=Math.cos(Range.clip(rotationController.getLastError(), -Math.PI/2, Math.PI/2));
        }
        //flip check
        else if(MOTOR_FLIPPING) power*=flipModifier();
        motor.setPower(power);
    }

    public boolean isWithinAllowedError(){
        double error = Math.abs(rotationController.getLastError());
        return error < ALLOWED_COS_ERROR || error > 2*Math.PI - ALLOWED_COS_ERROR;
    }

    public void setServoPower(double power) {
        servo.setPower(power);
    }

    public void setTargetRotation(double target) {
        double current = getModuleRotation();
        //normalize for wraparound
        if (current - target > Math.PI) current -= (2 * Math.PI);
        else if (target - current > Math.PI) current += (2 * Math.PI);

        if (MOTOR_FLIPPING) {
                //flip target
            wheelFlipped = Math.abs(current - target) > (Math.PI / 2 - flipModifier()*FLIP_BIAS);
            if (wheelFlipped) target = Angle.norm(target + Math.PI);
        }
        rotationController.setTargetPosition(target);
    }
    public SwerveModuleState asState(){
        return new SwerveModuleState(this);
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPos, podRot;
        public SwerveModuleState(SwerveModule s){
            module = s;
            wheelPos = 0;
            podRot = 0;
        }

        public SwerveModuleState update(){
            return setState(-module.getWheelPosition(), module.getModuleRotation());
        }
        public SwerveModuleState setState(double wheel, double pod){
            wheelPos = wheel;
            podRot = pod;
            return this;
        }
        //TODO add averaging for podrots based off of past values
        public Vector2d calculateDelta(){
            double oldWheel = wheelPos;
            update();
            return Vector2d.polar(wheelPos-oldWheel, podRot);
        }
    }
}
