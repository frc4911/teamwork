package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by rodney on 10/20/2018.
 */

public class RevMagTouchSensor {

    private LinearOpMode opmode;
    private HardwareTileRunner robot;

    private DigitalChannel digitalTouch;

    public RevMagTouchSensor() {
    }


    public void init(String sensorName, LinearOpMode opmode, HardwareMap hwMap) {
        this.opmode = opmode;
        this.robot = robot;

        digitalTouch = hwMap.get(DigitalChannel.class, sensorName);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        // getState() returns false when it is pressed
        // so we are using reverse logic to simplify it
        return !digitalTouch.getState();
    }
}