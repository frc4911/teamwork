package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by thomp on 10/7/2017.
 */

public class RevIMU {

    // RevIMU/Gyro - using built-in REV RevIMU which is a BNO055
    private BNO055IMU imu = null;

    public RevIMU() {
    }

    public void init(String sensorName, HardwareMap ahwMap) {
        imu = RevIMU.getIMU(ahwMap);
    }

    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // axes order is ZYX, Z axis is heading so first angle is heading.
    }

    public String formatHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle("Heading", angles.angleUnit, angles.firstAngle);
    }

    private String formatAngle(String caption, AngleUnit angleUnit, double angle) {
        return formatDegrees(caption, AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(String caption, double degrees){
        String msg = String.format(Locale.US, "-----IMU HEADING-----------------------%n");
        msg += String.format(Locale.getDefault(), "%s: %.1f", caption, AngleUnit.DEGREES.normalize(degrees));
        return msg;
    }

    //----------------------------------------------------------------------------------------------
    // This method is used to determine which team we are running as.
    // The IMU on the Rev has a unique ID.
    //
    // This method is static because it is called before we have a robot object, so it needs
    // to get access to the IMU separately.
    //
    public static byte[] getDeviceID(HardwareMap ahwMap) {

        BNO055IMU  bno055_IMU = RevIMU.getIMU(ahwMap);

        // TO READ A REGISTER ON THE IMU...
        // 1 - go to config mode, wait to be sure in CONFIG mode
        // 2 - Switch to appropriate register page and read
        // 3 - Read BNO ID registers
        // 4 - go back to IMU mode, wait to be sure in IMU mode
        // 5 - to back to register page 0

        // 1 - go to config mode, wait to be sure in CONFIG mode
        bno055_IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        while ( bno055_IMU.read8(BNO055IMU.Register.OPR_MODE) != BNO055IMU.SensorMode.CONFIG.bVal ) {
            // wait to be in config mode
        }

        // 2 - Switch to register page 0 and read
        bno055_IMU.write8(BNO055IMU.Register.PAGE_ID, 0x01);

        // 3 - Read BNO ID registers
        int cbBnoId = 16;
        byte[] result = bno055_IMU.read(BNO055IMU.Register.UNIQUE_ID_FIRST, cbBnoId);

        // 4 - go back to IMU mode, wait to be sure in IMU mode
        bno055_IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        while ( bno055_IMU.read8(BNO055IMU.Register.OPR_MODE) != BNO055IMU.SensorMode.IMU.bVal ) {
            // wait to be in config mode
        }

        // 5 - to back to register page 0
        bno055_IMU.write8(BNO055IMU.Register.PAGE_ID, 0x00);

        int[]  id = new int[result.length];

        for ( int i = 0 ; i < result.length ; i++ ) {
            id[i] = result[i];
        }
        return result;
    }

    public static String formatDeviceID(HardwareMap ahwMap) {

        byte[] chipIDarray = getDeviceID(ahwMap);

        // only need opmode to display the chip ID info ... this is temporary and for debugging
        String msg = String.format(Locale.US, "-----IMU ID----------------------------%n");
        for ( byte b : chipIDarray ) {
            msg += String.format(Locale.getDefault(), "[%d] ", b);
        }

        return msg;
    }

    private static BNO055IMU getIMU(HardwareMap ahwMap) {
        BNO055IMU  bno055_IMU;

        // Set up the parameters with which we will use our RevIMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "RevIMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        bno055_IMU = ahwMap.get(BNO055IMU.class, BotParams.IMU_SENSOR_NAME);
        bno055_IMU.initialize(parameters);

        return bno055_IMU;
    }

}
