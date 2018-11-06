package org.firstinspires.ftc.teamcode.robot.control.helpers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by thomp on 10/6/2017.
 */

public class RevColorSensor {

    private LinearOpMode opmode;
    private HardwareTileRunner robot;

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    private final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private final double SCALE_FACTOR = 255;

    public RevColorSensor() {
    }

    public void init(String sensorName, LinearOpMode opmode, HardwareMap hwMap) {
        this.opmode = opmode;
        this.robot = robot;

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, sensorName);

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, sensorName);
    }

    public int red() {
        return sensorColor.red();
    }

    public int blue() {
        return sensorColor.blue();
    }

    public int green() {
        return sensorColor.green();
    }

    public String formatColor() {
        String msg = String.format(Locale.US, "-----COLOR SENSOR ---------------------%n");
        msg += String.format(Locale.US, "Red:   %d%nGreen: %d%nBlue: %d", sensorColor.red(), sensorColor.green(), sensorColor.blue());
        return msg;
    }

    public String formatDistance() {
        String msg = String.format(Locale.US, "-----DISTANCE SENSOR-------------------%n");
        msg += String.format(Locale.US, "Distance (cm): %.02f", sensorDistance.getDistance(DistanceUnit.CM));
        return msg;
    }

    private void RGBToHSV() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
    }
}
