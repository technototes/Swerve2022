package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private Encoder.Direction inverted;

    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = Encoder.Direction.FORWARD;
    }
    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(Encoder.Direction invert){
        inverted = invert;
        return this;
    }
    public Encoder.Direction getDirection() {
        return inverted;
    }

    public double getCurrentPosition() {
        return encoder.getVoltage()*getDirection().getMultiplier()*Math.PI*2/2.3;
    }

}
