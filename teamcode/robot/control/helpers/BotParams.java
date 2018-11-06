package org.firstinspires.ftc.teamcode.robot.control.helpers;

/**
 * Created by thomp on 10/10/2017.
 */

public class BotParams {

    /*
    ** Defines team specific parameters.
    **
    ** Default values are provided here and it is up to the team specific code to change them as needed
    */

    //==============================================================================================
    // TEAM ID IS DETERMINED BY READING THE IMU DEVICE ID
    // This requires knowing the device name of the IMU in the robot system
    // Thus the name of the IMU is a static final so we can read the IMU before the robot is initialized.
    //
    public static final String IMU_SENSOR_NAME = "imu";

    //==============================================================================================
    // TEAM ID
    //
    public int teamNumber;
    public String teamName;

    //==============================================================================================
    // DRIVE SYSTEM PARAMETERS
    //

    // device names for the drive motors
    public String leftDriveMotorName = "left_drive";
    public String rightDriveMotorName = "right_drive";

    // Gear-motor output shaft encoder tick count per revolution
    public static final int TETRIX_ENCODER_OUTPUT_COUNTS_PER_REV      = 1440;
    public static final int NEVEREST_60_ENCODER_OUTPUT_COUNTS_PER_REV = 1680;
    public static final int NEVEREST_40_ENCODER_OUTPUT_COUNTS_PER_REV = 1120;
    public static final int NEVEREST_20_ENCODER_OUTPUT_COUNTS_PER_REV = 560;

    public double countsPerMotorRev = NEVEREST_40_ENCODER_OUTPUT_COUNTS_PER_REV;

    // Gear box reduction ... typically use 45 and 35 tooth gears
    // This is < 1.0 if geared UP
    public double driveGearReduction = 35.0/45.0;

    // Have typically used 4" wheels
    public double wheelDiameterInches = 4.0;

    // Calcuated based on above params (countsPerMotorRev, driveGearReduction, wheelDiameterInched)
    // Method calcContsPerInch() is used to performm this calculation
    public double countsPerInch;

    public String armExtendMotorName = "arm_extend";
    public static final int ARM_EXTEND_SOFT_MAX_OUT_POS = 11510;
    public static final int ARM_EXTEND_SOFT_MAX_IN_POS = 400;
    public static final double ARM_EXTEND_SPEED_SCALING = 1; // cut the motor speed for better control

    public double speedBoostFactor = 2;
    public double regularDriveFactor = -0.5;
    public double extendJoystickFactor = 1;

    // All chassis drive differently, some pull to the left, others the right and a rare few drive straight.
    // The following variables all us to adjust for pull.
    // If the robot drives straight, both scaling factors will be the same (likely 1.0).
    // If the robot pulls to the left, the right side is overpowering the left so the left scale needs to be greater


    public double leftSideSpeedScale = 1.0;
    public double rightSideSpeedScale = 1.0;

    // Gyro drive settings
    public double gyroDriveHeadingThreshold = 2.0;   // accuracy tolerance
    public double gyroDriveTurnPCoeff       = 0.15;   // PID proportional coefficient for turning...larger is more responsive
    public double gyroDriveDrivePCoeff      = 0.05;  // PID proportional coefficient for driving straight...larger more responsive

    //==============================================================================================
    // SENSOR PARAMETERS
    //

    // ...IMU is accessed statically as we use it to determine which team to run as (i.e. which
    // ...specific hardware class in instantiate).  So we are not using a variable for tha IMU name.
    //public String imuSensorName = IMU_SENSOR_NAME;

    public static final String UPPER_ARM_SENSOR_NAME = "sensor_arm_upper";
    public static final String LOWER_ARM_SENSOR_NAME = "sensor_arm_lower";

    //==============================================================================================
    // SERVO PARAMETERS
    //

    public String markerTurretName = "turret";
    public double markerTurretStow = -0.3;
    public double markerTurretIn = -0.3;
    public double markerTurretOut = 0.3;

    public String latchName = "latch";
    public double latchStow = 0.3;
    public double latchOpen = -0.3;
    public double latchClosed = 0.3;

    //==============================================================================================
    // AUTONOMOUS PARAMETERS
    private static final double AUTO_DRIVE_SPEED = .35;
    private static final double AUTO_TURN_SPEED  = .35;
    // ---------------------------------------------------------------------------------------------
    // Method to calculate the number of encoder tick counts required to drive one inch.
    //
    public void calcCountsPerInch() {
        countsPerInch = (countsPerMotorRev / driveGearReduction) / (wheelDiameterInches * Math.PI);
    }
}
