package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by thomp on 6/30/2017.
 */

@Disabled
public class GyroDrive {

    /*
    ** THIS ASSUMES A TANK DRIVE WITH A SINGLE MOTOR ON EACH DRIVE SIDE.
    ** IF THERE ARE TWO MOTORS ON EACH DRIVE SIDE, THEY WILL NEED TO BE ON THE SAME MOTOR PORT.
     */

    LinearOpMode opmode;
    HardwareTileRunner robot;

    private ElapsedTime stopwatch = new ElapsedTime();

    public GyroDrive() {
    }

    public void init(LinearOpMode currRunningOpmode, HardwareTileRunner robot) {
        this.opmode = currRunningOpmode;
        this.robot = robot;
    }

    public void stop() {
        robot.driveBothSidesAuto(0,0);
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired currPosition
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void turn ( double speed,
                        double angle) {

        double  max;
        double  error = getError(angle);
        double  leftSpeed;
        double  rightSpeed;
        double speedSlowFactor;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            boolean errorStartedPositive = false;

            speed = Math.abs(speed);
            if (speed > 1) {
                speed = 1;
            }

            if (error > 0) {
                errorStartedPositive = true;
            } else if (error < 0) {
                errorStartedPositive = false;
            }
            do {
                // adjust relative speed based on heading error.
                error = getError(angle);
                speedSlowFactor = error / 45.0;
                double newSpeed = Math.min(speedSlowFactor,speed);
                if(!errorStartedPositive) {
                    newSpeed *= -1;
                }
                leftSpeed = newSpeed;
                rightSpeed = -newSpeed;

                robot.driveBothSidesAuto(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                opmode.telemetry.addData("Heading", "%5.3f", robot.imu.getHeading());
                opmode.telemetry.addData("Error", "%5.1f", error);
                opmode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                opmode.telemetry.addData("Pcoeff", "%5.3f", robot.getBotParms().gyroDriveDrivePCoeff);
                opmode.telemetry.update();

            } while (((errorStartedPositive && error > 0) || (!errorStartedPositive && error < 0)) && opmode.opModeIsActive());
                // Stop all motion;
                stop();

                // Turn off RUN_TO_POSITION
                robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void hold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opmode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, robot.getBotParms().gyroDriveTurnPCoeff);
            opmode.telemetry.update();
        }

        // Stop all motion;
        stop();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.getBotParms().gyroDriveHeadingThreshold) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed    = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.driveBothSidesAuto(leftSpeed,rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Heading", "%5.2f", robot.imu.getHeading());
        opmode.telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opmode.telemetry.addData("Pcoeff", "%5.3f", robot.getBotParms().gyroDriveTurnPCoeff);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn CRATER_NO_MARKER (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getSteerNoClip(double error, double PCoeff) {
        return error * PCoeff;
    }

}
