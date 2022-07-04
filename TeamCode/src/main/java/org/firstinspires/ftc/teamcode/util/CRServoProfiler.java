package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class CRServoProfiler {
    public static double TOLERANCE = 0.02, POS_ESTIMATE = 2.3;

    public static class Constraints {
        public double maxVel, maxAccel, maxJerk;

        public Constraints(double vel, double accel, double jerk) {
            maxVel = vel;
            maxAccel = accel;
            maxJerk = jerk;
        }

        public double getMaxVel() {
            return maxVel;
        }

        public double getMaxAccel() {
            return maxAccel;
        }

        public double getMaxJerk() {
            return maxJerk;
        }
    }

    private CRServo device;
    private Constraints constraints;
    private ElapsedTime time;
    public double targetVel, currentPos, currentVel, currentAccel, currentJerk;

    public CRServoProfiler(CRServo device, Constraints constraints) {
        this.device = device;
        this.constraints = constraints;
        time = new ElapsedTime();
        currentVel = 0;
        currentPos = 0;
        targetVel = 0;
    }

    public CRServoProfiler setTarget(double target) {
        targetVel = target;
        update();
        return this;
    }


    public CRServoProfiler update() {
        //if at the target dont do anything
        double pastVel = currentVel, pastAccel = currentAccel, pastJerk = currentJerk, deltaSec = Math.min(time.seconds(), 0.05);
        currentPos += 2 * Math.PI * POS_ESTIMATE * deltaSec * getMeasured();
        time.reset();
        if (isAtTarget()) {
            currentVel = targetVel;
            currentAccel = 0;
            currentJerk = 0;

        } else {
//            double stopVelocity = 0.5 * currentAccel * Math.abs(currentAccel) / constraints.getMaxJerk();
//            System.out.println("currentVelocity "+currentVel+ " target "+targetVel);
            currentJerk = Math.signum(getError()) * constraints.getMaxJerk();
            currentAccel = Range.clip(currentAccel + deltaSec * (currentJerk + pastJerk) / 2, -constraints.getMaxAccel(), constraints.getMaxAccel());
            currentVel = Range.clip(currentVel + deltaSec * (currentAccel + pastAccel) / 2, -constraints.getMaxVel(), constraints.getMaxVel());
//            System.out.println(currentPos);

            //        System.out.println(currentVel);
        }

        device.setPower(currentVel);
//        device.setPower(targetVel);
        return this;
    }

    public double getMeasured() {
        return device.getPower();
    }

    public boolean isAtTarget() {
        return Math.abs(getError()) < TOLERANCE;
    }

    public double getError() {
        return getTarget() - getMeasured();
    }

    public double getTarget() {
        return targetVel;
    }

}
