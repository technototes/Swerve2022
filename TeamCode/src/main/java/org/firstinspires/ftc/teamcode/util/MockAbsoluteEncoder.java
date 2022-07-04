package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;

import java.util.function.DoubleSupplier;

public class MockAbsoluteEncoder extends AnalogInput {
    private ElapsedTime time;

    public MockAbsoluteEncoder() {
        super(null, 0);
        time = new ElapsedTime();
    }

}
