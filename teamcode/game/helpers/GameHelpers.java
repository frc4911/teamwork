package org.firstinspires.ftc.teamcode.game.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.control.helpers.BotParams;
import org.firstinspires.ftc.teamcode.robot.control.helpers.HardwareTileRunner;

/**
 * Created by thomp on 10/13/2017.
 * This class provides game specific parametric and action helpers.
 *
 */

public class GameHelpers {
    public enum AllianceColor    { UNKNOWN(0), RED(1), BLUE(2);      public final byte bVal; AllianceColor(int i) { bVal =(byte)i; }}
    public enum SamplePos { LEFT(1), CENTER(2), RIGHT(3);     public final byte bVal; SamplePos(int i) { bVal =(byte)i; }}
    public enum StartingCorner { CRATER_NO_MARKER(1), CRATER_MARKER(2), DEPOT(3);  public final byte bVal; StartingCorner(int i) { bVal =(byte)i; }}
    public AllianceColor allianceColor = null;
    public StartingCorner startingCorner = null;
    public BotParams botParams;

    HardwareTileRunner robot;
    LinearOpMode       opmode;

    // ---------------------------------------------------------------------------------------------
    // Constructor ... put no code here!  Must use the init() method.
    public GameHelpers() {}
    // ---------------------------------------------------------------------------------------------
    // Initialize the helper subsystem.
    // Must be called after the robot has been initialized.
    //
    public void init(LinearOpMode currRunningOpmode, HardwareTileRunner robot) {
        this.robot = robot;
        botParams = robot.getBotParms();
        this.opmode = currRunningOpmode;
        robot.armReset();
    }

    public void initWaitForStart() {
    }

    public void initRun() {
        robot.imu.start();
    }

//==================Autonomous======================================================================

    public void runAutonomous(StartingCorner startingCorner) {
        // set startingCorner fields
        this.startingCorner = startingCorner;
        //run auto
        switch (startingCorner) {
            case DEPOT:
                runDepotAutonomous();
                break;
            case CRATER_NO_MARKER:
                runCraterAutonomousNoMarker();
                break;
            case CRATER_MARKER:
                runCraterAutonomousMarker();
                break;
        }
    }

    private void runDepotAutonomous() {
        lower();
        robot.encoderDrive.driveStraight(1.0, -59, 7);
        robot.gyroDrive.hold(0.35, -45.0, 5.0);
        robot.markerTurretOut();
        opmode.sleep(125);
        robot.markerTurretIn();
        robot.encoderDrive.driveStraight(1.0, 75, 10);
    }

    private void runCraterAutonomousNoMarker() {
        lower();
        robot.encoderDrive.driveStraight(1.0,-31,5);
    }

    private void runCraterAutonomousMarker() {
        lower();
        robot.encoderDrive.driveStraight(0.5, -3, 3);
        robot.gyroDrive.hold(0.35, 45, 4);
        robot.encoderDrive.driveStraight(1.0, -35, 5);
        robot.gyroDrive.hold(0.35,-45, 4);
        robot.encoderDrive.driveStraight(1.0,73, 10);
        opmode.sleep(500);
        robot.markerTurretOut();
        opmode.sleep(125);
        robot.markerTurretIn();
        robot.encoderDrive.driveStraight(1.0,-80,10);
    }

    private void lower() {
        robot.autoArmExtend(1.0, botParams.ARM_EXTEND_SOFT_MAX_OUT_POS, 7.0);
        robot.openLatch();
        opmode.sleep(1000);
    }

    public void lowerPublic() {
        robot.autoArmExtend(1.0, botParams.ARM_EXTEND_SOFT_MAX_OUT_POS, 7.0);
        robot.openLatch();
    }

        private SamplePos determineSamplePosition() {
        return SamplePos.CENTER; //default
    }

}
