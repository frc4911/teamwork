package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * It provides methods to use and manipulate all devices on the robot.
 *
 * No initialization work can be done in the constructor, it must all be done in
 * the init() method.  This is because we need a reference to the hardware map
 * and the opmode in order to function and those do not exist at construction time.
 *
 * This class is extended by a team specific class that allows teams to override
 * behavior and parameters.  The team specific hardware class init() method call
 * this (the super class) class's init().
 *
 * All parameters are in the BotParams class and can be overridden in the team specific
 * hardware class.
 *
 */
public abstract class HardwareTileRunner {

    private LinearOpMode opmode;

    /* Drive system */
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;

    // arm motors
    //public DcMotor armRotate = null;
    private DcMotor armExtend = null;

    // RevIMU/Gyro
    public RevIMU imu = new RevIMU();

    //Arm Touch Sensors
    public RevMagTouchSensor upperArmLimit = new RevMagTouchSensor();
    public RevMagTouchSensor lowerArmLimit = new RevMagTouchSensor();

    // Drive control subsystems
    public GyroDrive gyroDrive = new GyroDrive();
    public TimedDrive   timedDrive = new TimedDrive();
    public EncoderDrive encoderDrive = new EncoderDrive();

    // Servos and associated state ... we are not exposing the actual servo outside this class as no one
    // using these should directly drive them ... must go throught the ServoControl interface.\
//    private ServoControl markerTurret = new ServoControl();
    private ServoControl latch = new ServoControl();
    private ServoControl markerTurret = new ServoControl();

    private BotParams botParms;

    /* Other */
    private boolean extendLimits = true;

    /* Constructor */
    public HardwareTileRunner(){
    }

    /* Initialize standard Hardware interfaces */
    public abstract void init(LinearOpMode opmode, HardwareMap ahwMap);

    public void init(LinearOpMode runningOpmode, HardwareMap ahwMap, BotParams botParms) {
        this.opmode = runningOpmode;
        this.botParms = botParms;

        this.botParms.calcCountsPerInch();

        setUpDriveSystem(ahwMap);
        setUpArmSystem(ahwMap);
        setUpSensors(ahwMap);
        setUpServos(ahwMap);
    }

    public void driveBothSidesArcade(double drive, double turn, boolean boost) {
        // Combine drive and turn for blended motion.
        double left  = drive + turn;
        double right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        // Output the safe vales to the motor drives.
        driveBothSidesTank(left,right,boost);
    }

    public void driveBothSidesTank(double leftDrivePower, double rightDrivePower, boolean boost) {
        double boostBy = 1;
        if (boost) {
            boostBy = botParms.speedBoostFactor;
        }
        double powerLeft = leftDrivePower * botParms.regularDriveFactor * boostBy;
        double powerRight = rightDrivePower * botParms.regularDriveFactor  * boostBy;
        driveBothSidesAuto(powerLeft,powerRight);
    }

    public void driveBothSidesAuto(double leftDrivePower, double rightDrivePower) {
        double power = leftDrivePower * getLeftSideSpeedScale();
        leftDrive.setPower(power);
        opmode.telemetry.addData("left", "%.2f", power);
        power = rightDrivePower * getRightSideSpeedScale();
        rightDrive.setPower(power);
        opmode.telemetry.addData("right", "%.2f", power);
        String tel = "left_encoder "+leftDrive.getCurrentPosition();
        opmode.telemetry.addData("left_encoder", tel);
//        opmode.telemetry.addData("right_encoder", "%.2d", rightDrive.getCurrentPosition());
        tel = "right_encoder "+rightDrive.getCurrentPosition();
        opmode.telemetry.addData("right_encoder", tel);
        opmode.telemetry.update();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftDrive.setZeroPowerBehavior(zeroPowerBehavior);
        rightDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setTargetPositions(int left, int right) {
        leftDrive.setTargetPosition(left);
        rightDrive.setTargetPosition(right);
    }

    //TELEOP ONLY!!!
    public void teleopArmExtend(double extendPower) {
        extendPower *= botParms.ARM_EXTEND_SPEED_SCALING * botParms.extendJoystickFactor;
        //TODO: change when limits are on robot
        if(extendLimits && excedingSoftLimits(extendPower)) {
            extendPower = 0;
        }
        opmode.telemetry.addData("Arm Position",getExtendCurrentPosition());
        setArmPower(extendPower);
    }

    public void setArmPower(double extendPower) {
        armExtend.setPower(extendPower);
    }

    public void autoArmExtend(double power, int goalPosition, double timeOutmSecs) {
        ElapsedTime stopwatch = new ElapsedTime();

        // clip arm power
        power = Range.clip(power, -1.0, 1.0);

        // move the arm
        armExtend.setTargetPosition(goalPosition);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stopwatch.reset();

        armExtend.setPower(power);

        // keep looping while we are still active, and there is time left, both motors are running, and it hasn't been cancelled.
        //TODO: change when limits are on robot
        while (opmode.opModeIsActive() && stopwatch.seconds() < timeOutmSecs && armExtend.isBusy() && !excedingSoftLimits(power)) {}

        // Stop all motion;
        armExtend.setPower(0);

        // Turn off RUN_TO_POSITION
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //True if the extend value excedes the soft limit and continues to try to exceed it further
    private boolean excedingSoftLimits(double extendPower) {
        return (getExtendCurrentPosition() > botParms.ARM_EXTEND_SOFT_MAX_OUT_POS && extendPower > 0) ||
                (getExtendCurrentPosition() < botParms.ARM_EXTEND_SOFT_MAX_IN_POS && extendPower < 0) ;
    }

    //TODO: Use BOTH touch sensors
    private boolean excedingHardLimits(double extendPower) {
        return (upperArmLimit.isPressed() && extendPower > 0);// || (lowerArmLimit.isPressed() && extendPower < 0);
    }

    //True if the value excedes a limit, hard or soft
    private boolean excedingLimits(double extendPower) {
        return excedingSoftLimits(extendPower) || excedingHardLimits(extendPower);
    }

    private void setUpDriveSystem(HardwareMap ahwMap) {

        // Define and Initialize Motors
        leftDrive = ahwMap.dcMotor.get(botParms.leftDriveMotorName);
        rightDrive = ahwMap.dcMotor.get(botParms.rightDriveMotorName);

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // init drive control subsystems ... comment out unused
        timedDrive.init(opmode, this);
        encoderDrive.init(opmode, this);
        gyroDrive.init(opmode, this);
    }

    private void setUpArmSystem(HardwareMap ahwMap) {

        // Define and Initialize Motors
        //armRotate = ahwMap.dcMotor.get(botParms.armRotateMotorName);
        armExtend = ahwMap.dcMotor.get(botParms.armExtendMotorName);

        armExtend.setDirection(DcMotor.Direction.FORWARD);
//        armExtend.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        //armRotate.setPower(0);
        armExtend.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setUpSensors(HardwareMap ahwMap) {
        imu.init(BotParams.IMU_SENSOR_NAME, ahwMap);
//        upperArmLimit.init(botParms.UPPER_ARM_SENSOR_NAME, opmode, ahwMap);
//        lowerArmLimit.init(botParms.LOWER_ARM_SENSOR_NAME, opmode, ahwMap);
    }

    private void setUpServos(HardwareMap ahwMap) {
//        markerTurret.init(botParms.markerTurretName, botParms.markerTurretStow, opmode, ahwMap);
        latch.init(botParms.latchName, botParms.latchStow, opmode, ahwMap);
        markerTurret.init(botParms.markerTurretName, botParms.markerTurretStow, opmode, ahwMap);
    }

    public void armReset() {
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void latchStow() {
        latch.setPosition(botParms.latchStow);
    }

    public void openLatch() {latch.setPosition(botParms.latchOpen); }

    public void closeLatch() { latch.setPosition(botParms.latchClosed); }

    public void marketTurretStow() {
        markerTurret.setPosition(botParms.markerTurretStow);
    }

    public void markerTurretIn() {markerTurret.setPosition(botParms.markerTurretIn); }

    public void markerTurretOut() {
        markerTurret.setPosition(botParms.markerTurretOut);
        opmode.sleep(1000);
    }

    public double getCountsPerInch() {
        return botParms.countsPerInch;
    }

    public double getLeftSideSpeedScale() {
        return botParms.leftSideSpeedScale;
    }

    public double getRightSideSpeedScale() {
        return botParms.rightSideSpeedScale;
    }

    public BotParams getBotParms() {
        return botParms;
    }

    public void setExtendLimits(boolean flag) {
        extendLimits = flag;
    }

    public int getExtendCurrentPosition() {
        return armExtend.getCurrentPosition();
    }
}

