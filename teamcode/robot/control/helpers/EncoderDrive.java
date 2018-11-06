package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Created by thomp on 6/30/2017.
 */

@Disabled
public class EncoderDrive {

    /*
    ** THIS ASSUMES A TANK DRIVE WITH A SINGLE MOTOR ON EACH DRIVE SIDE.
    ** IF THERE ARE TWO MOTORS ON EACH DRIVE SIDE, THEY WILL NEED TO BE ON THE SAME MOTOR PORT.
     */

    LinearOpMode opmode;
    HardwareTileRunner robot;

    private ElapsedTime stopwatch = new ElapsedTime();

    public EncoderDrive() {
    }

    public void init(LinearOpMode currRunningOpmode, HardwareTileRunner robot) {
        this.opmode = currRunningOpmode;
        this.robot = robot;
    }

    public void stop() {
        robot.driveBothSidesAuto(0,0);
    }

    public void resetEncoders() {
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMotorsToBrakeMode();
    }

    public void setDriveMotorsToBrakeMode() {
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDriveMotorsToCoastMode() {
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /*
     *  Method to drive straight, based on encoder counts.
     *  Encoders are not reset as the move is based on the current currPosition.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired currPosition
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void driveStraight(double speed, double distInches, double timeoutS) {
        drive(speed, distInches, distInches, timeoutS);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current currPosition.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired currPosition
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void drive(double speed,
                      double leftInches, double rightInches,
                      double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            leftInches *= -1;
            rightInches *= -1;

            // Determine new target currPosition, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * robot.getCountsPerInch());
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * robot.getCountsPerInch());

            robot.setTargetPositions(newLeftTarget, newRightTarget);
            // Turn On RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            stopwatch.reset();

            // drive
            robot.driveBothSidesAuto(Math.abs(speed),Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opmode.opModeIsActive() &&
                    stopwatch.seconds() < timeoutS &&
                    (robot.leftDrive.isBusy() || robot.rightDrive.isBusy() )
                    ) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                opmode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                opmode.telemetry.update();
            }

            // Stop all motion;
            stop();

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public String formatEncoders() {
        String msg = String.format(Locale.US, "-----DRIVE ENCODERS--------------------%n");
        //msg += String.format(Locale.US, "Counts/Inch: %.2f%n", robot.getBotParms().countsPerInch);
        //msg += String.format(Locale.US, "     Counts/Motor Rev: %.2f%n", robot.getBotParms().countsPerMotorRev);
        //msg += String.format(Locale.US, "     Drive Gear Reduction: %.2f%n", robot.getBotParms().driveGearReduction);
        //msg += String.format(Locale.US, "     Wheel Diam/Inch: %.2f%n", robot.getBotParms().wheelDiameterInches);
        msg += String.format(Locale.US, "Left: %d%n", robot.leftDrive.getCurrentPosition());
        msg += String.format(Locale.US, "Right: %d", robot.rightDrive.getCurrentPosition());
        return msg;
    }

}
