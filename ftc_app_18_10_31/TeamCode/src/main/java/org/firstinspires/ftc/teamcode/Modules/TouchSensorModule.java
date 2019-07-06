package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by rsf on 2/25/19.
 */

public class TouchSensorModule {
    private DigitalChannel digitalTouch = null;

    public void init(OpMode opMode) {
        // get a reference to our digitalTouch object.
        digitalTouch = opMode.hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean loop() {
        return digitalTouch.getState();
    }
}
