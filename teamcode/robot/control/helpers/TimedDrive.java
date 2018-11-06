package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by thomp on 6/30/2017.
 */

//@Disabled
public class TimedDrive {

    /*
    ** THIS ASSUMES A TANK DRIVE WITH A SINGLE MOTOR ON EACH DRIVE SIDE.
    ** IF THERE ARE TWO MOTORS ON EACH DRIVE SIDE, THEY WILL NEED TO BE ON THE SAME MOTOR PORT.
     */

    LinearOpMode opmode;
    HardwareTileRunner robot;

    private ElapsedTime stopwatch = new ElapsedTime();

    public TimedDrive() {

    }

    public void init(LinearOpMode opMode, HardwareTileRunner robot) {
        this.opmode = opmode;
        this.robot = robot;
    }


    public void stop() {
        robot.driveBothSidesTank(0,0,false);
    }

    /*
     *  Method to perform a relative move driving straight, based on time.
     *  Move will stop if any of three conditions occur:
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void drive(double speed,
                      double timeoutS) {

        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.driveBothSidesTank(speed,speed,false);
        stopwatch.reset();
        while (opmode.opModeIsActive() && (stopwatch.seconds() < timeoutS)) {
            opmode.telemetry.addData("DriveStraight", "%2.5f S Elapsed", stopwatch.seconds());
            opmode.telemetry.update();
        }
    }

    /*
     *  Method to perform a relative move using a pivot turn, based on time.
     *  Move will stop if any of three conditions occur:
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void turn(double speed,
                     boolean turnLeft, // true means turn left, false turn right
                     double timeoutS) {

        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        String direction = "";
        if ( turnLeft  ) {
            robot.driveBothSidesTank(-speed,speed,false);
            direction = "left";
        } else {
            robot.driveBothSidesTank(speed,-speed,false);
            direction = "right";
        }
        stopwatch.reset();
        while (opmode.opModeIsActive() && (stopwatch.seconds() < timeoutS)) {
            opmode.telemetry.addData("DriveTurn", "%s (%3.2f): %2.5f S Elapsed", direction, speed, stopwatch.seconds());
            opmode.telemetry.update();
        }

        stop();
    }


}
