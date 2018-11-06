package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by thomp on 10/3/2017.
 */

public class ServoControl {

    private static final double INCREMENT   = 0.04;     // amount to slew servo each CYCLE_MS cycle
    private static final int    CYCLE_MS    = 100; // =   50;     // period of each cycle

    // Define class members
    private LinearOpMode opmode;

    private Servo servo;
    private String servoName;
    private double currPosition;
    private boolean rampUp = true;

    private double newPosition;
    private boolean atNewPosition;

    public ServoControl() {
    }

    public void init(String servoName, double startPos, LinearOpMode opmode, HardwareMap ahwMap) {
        this.opmode = opmode;
        this.servoName = servoName;

        servo = ahwMap.servo.get(servoName);
        setPosition(startPos); // sets currPosition
    }

    //-------------------------------------------------------------------------------------------
    // Ramp to position methods
    // This process is broken into "iterative" opmode format with init(), loop(), and stop() type methods
    // that are called to from a loop.
    // The method rampToPosition() has a loop using these methods.
    // The rampLoopInit(), rampLoopIsDone(), rampLoopIterationStep() and rampLoopStop() methods can be used
    // in an OpMode (iterative opmode).
    public void rampToPosition(double newPosition) {

        rampLoopInit(newPosition);
        while ( !rampLoopIsDone() ) {
            rampLoopIterationStep();
        }
        rampLoopStop();
    }

    public void rampLoopInit(double newPosition) {
        this.newPosition = newPosition;

        opmode.telemetry.addData("ServoControl:", "goal pos %5.2f", newPosition);

        rampUp = false;
        if ( currPosition == newPosition ) {
            opmode.telemetry.addData("ServoControl:", "Done");
            opmode.telemetry.update();
            return;
        }
        else if ( currPosition < newPosition ) {
            rampUp = true;
        }

        // Scan servo till stop pressed.
        atNewPosition = false;
    }

    public boolean rampLoopIsDone() {
        return (atNewPosition || !opmode.opModeIsActive());
    }

    public void rampLoopIterationStep() {
        // slew the servo, according to the rampUp (direction) variable.
        if (rampUp) {
            // Keep stepping up until we hit the max value.
            currPosition += INCREMENT ;
            if (currPosition >= newPosition ) {
                currPosition = newPosition;
                atNewPosition = true;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            currPosition -= INCREMENT ;
            if (currPosition <= newPosition ) {
                currPosition = newPosition;
                atNewPosition = true;
            }
        }

        // Display the current value
        opmode.telemetry.addData("ServoControl:", "curr pos %5.2f", currPosition);
        //telemetry.update();

        // Set the servo to the new currPosition and pause;
        setPosition(currPosition);
        opmode.sleep(CYCLE_MS);
        opmode.idle();
    }

    public void rampLoopStop() {
        // Signal done;
        opmode.telemetry.addData("ServoControl:", "Done");
        //opmode.telemetry.update();
    }

    //-------------------------------------------------------------------------------------------
    // This method simply records the new position as our current
    // and instructs the servo to drive to that position.
    // It does no ramping.
    public void setPosition(double position) {
        this.currPosition = position;
        servo.setPosition(position);
    }

}
