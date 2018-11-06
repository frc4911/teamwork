package org.firstinspires.ftc.teamcode.game.helpers;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robot.control.helpers.HardwareTileRunner;
//
///**
// * Created by thomp on 10/13/2017.
// * This class provides game specific parametric and action helpers.
// *
// */
//
public class GameHelpers2017 {
//    public enum AllianceColor    { UNKNOWN(0), RED(1), BLUE(2);      public final byte bVal; AllianceColor(int i) { bVal =(byte)i; }}
//    public enum SamplePos { CRATER_NO_MARKER(1), CENTER(2), DEPOT(3);     public final byte bVal; SamplePos(int i) { bVal =(byte)i; }}
//    public enum StartingCorner { LOWER(1), UPPER(2);                public final byte bVal; StartingCorner(int i) { bVal =(byte)i; }}
//
//    HardwareTileRunner robot;
//    LinearOpMode       opmode;
//
//    // ---------------------------------------------------------------------------------------------
//    // Constructor ... put no code here!  Must use the init() method.
//    public GameHelpers2017() {}
//    // ---------------------------------------------------------------------------------------------
//    // Initialize the helper subsystem.
//    // Must be called after the robot has been initialized.
//    //
//    public void init(LinearOpMode currRunningOpmode, HardwareTileRunner robot) {
//        this.robot = robot;
//        this.opmode = currRunningOpmode;
//    }
//
//    public void initWaitForStart() {
//        robot.clawRelicGrab();
//    }
//
//    public void initRun() {
//        robot.imu.start();
//    }
//
//    public void runAutonomous(AllianceColor allianceColor, StartingCorner corner) {
//        // read the picture and determine which glyph column to score in
//        SamplePos sample = determineGlyphColumn();
//
//        // score the jewel
//        scoreJewel(allianceColor);
//
//        // score the glyph
//        scoreGlyph(allianceColor, corner, sample);
//
//        // park
//        parkInCipherboxZone();
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // Score the jewel
//    // Bring the elbow down, use the color sensor to read the color of the left hand ball.
//    // If the ball color is the same as the alliance color, sweep right.
//    // If the ball color is different, sweep left.
//    //
//    public void scoreJewel(AllianceColor allianceColor) {
//
//        robot.elbowDown();
//
//        opmode.sleep( 500 );
//
//        // use color sensor to determine which way to go...
//        GameHelpers.AllianceColor ballColor = determineBallColor();
//        String colorMsg = formatBallColor(ballColor);
//
//        // the color sensor is looking at the left hand ball
//        // this assumes we are red alliance ... we want to knock off the blue ball
//        // so if the left ball is red, we want to move the jewel arm right.
//        boolean right = false;
//        if ( ballColor == GameHelpers.AllianceColor.RED ) {
//            right = true;
//            colorMsg += " (right)";
//        } else {
//            right = false;
//            colorMsg += " (left)";
//        }
//        opmode.telemetry.addData("scoreJewel:", colorMsg);
//        opmode.telemetry.update();
//
//        if ( right ) {
//            robot.turretRight();
//        } else {
//            robot.turretLeft();
//        }
//
//        robot.elbowUp();
//        robot.turretCenter();
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // Use the color sensor to determine the ball color.
//    // Assumes the color sensor is pointed at the ball the right distance away.
//    private GameHelpers.AllianceColor determineBallColor() {
//        if ( robot.colorSensor.red() - robot.colorSensor.blue() > 25 ) {
//            return GameHelpers.AllianceColor.RED;
//        } else if ( robot.colorSensor.blue() - robot.colorSensor.red() > 25 ) {
//            return GameHelpers.AllianceColor.BLUE;
//        }
//        return GameHelpers.AllianceColor.UNKNOWN;
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // Get a string representation of the ball color ... used for telemetry
//    public String formatBallColor(GameHelpers.AllianceColor ballColor) {
//        if ( ballColor == GameHelpers.AllianceColor.RED ) {
//            return "RED";
//        } else if ( ballColor == GameHelpers.AllianceColor.BLUE ) {
//            return "BLUE";
//        }
//        return "UNKNOWN";
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // Use the phone camera to determine which cipherbox column is the hot column
//    //
//    public SamplePos determineGlyphColumn() {
//        // insert code here to use the phone camera to scan the picture and determine which one is there
//        return SamplePos.CENTER;
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // Park in the cipherbox zone.
//    // Assume have just scored the glyph and hence the bot is facing the box centered on a column.
//    //
//    public void parkInCipherboxZone() {
//        // determine if we need to do anything
//    }
//
//    // ---------------------------------------------------------------------------------------------
//    // The process of scoring a glyph depends on 1) which alliance you are and 2) which balancing
//    // stone you start on.
//    //
//    // Balancing stones are referenced as left or right FROM THE DRIVE TEAM PERSPECTIVE.
//    //
//    // There are two basic patterns:
//    //    1) The "straight shot" ... just drive off the stone and turn to face the cipherbox.
//    //       ...blue alliance left stone
//    //       ...red alliance right stone
//    //    2) The "offset maneuver" ... drive off the stone, turn 90, move in front of the column, turn 90
//    //       ...blue alliance right stone
//    //       ...red alliance left stone
//    // The red alliance patterns are the reverse from the blue alliance patterns.
//    // ...on the red alliance the bot drives backwards off the stone, blue drives forwards.
//    // ...and also the red second turn is reversed for the offset maneuver.
//    //
//    // RED ALLIANCE DEPOT STONE - offset maneuver
//    //    1) drive FORWARD off balancing stone (distance based on which balancing stone we are on)
//    //    2) turn left 90 degrees (start heading is 0, final heading is 90)
//    //    3) drive forward to center robot on indicated glyph column (based on which balance stone...)
//    //    4) turn DEPOT 90 degrees to face the glyph column (start heading 90, final heading 0)
//    //    5) place glyph (***NOT DEFINED YET***)
//    //
//    // RED ALLIANCE CRATER_NO_MARKER STONE - straight shot
//    //    1) drive forward off balancing stone (distance based on which balancing stone we are on)
//    //    2) turn right 90 degrees to face the glyph column (start heading 0, final heading -90)
//    //    3) place glyph (***NOT DEFINED YET***)
//    //
//    // BLUE ALLIANCE DEPOT STONE - straight shot
//    //    1) drive BACKWARD off balancing stone (distance based on which balancing stone we are on)
//    //    2) turn right 90 degrees to face the glyph column (start heading 0, final heading -90)
//    //    3) place glyph (***NOT DEFINED YET***)
//    //
//    // BLUE ALLIANCE CRATER_NO_MARKER STONE - offset maneuver
//    //    1) drive BACKWARD off balancing stone (distance based on which balancing stone we are on)
//    //    2) turn left 90 degrees (start heading 0, final heading -90)
//    //    3) drive forward to center robot on indicated glyph column (based on which balance stone...)
//    //    4) turn CRATER_NO_MARKER 90 degrees to face the glyph column (start heading 90, final heading 180)
//    //    5) place glyph (***NOT DEFINED YET***)
//    //
//
//    private static final double DRIVE_SPEED = .35; // move these to botParms?
//    private static final double TURN_SPEED  = .35;
//
//    public void scoreGlyph(AllianceColor alliance, StartingCorner stone, SamplePos column) {
//        if ( alliance == AllianceColor.BLUE ) {
//            if ( stone == StartingCorner.LOWER ) {
//                scoreGlyphBlueLeft(column);
//            } else {
//                scoreGlyphBlueRight(column);
//            }
//        } else {
//            if ( stone == StartingCorner.LOWER ) {
//                scoreGlyphRedLeft(column);
//            } else {
//                scoreGlyphRedRight(column);
//            }
//        }
//    }
//
//    public void scoreGlyphRedRight(SamplePos column) {
//        // 1) drive forward off the stone
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, robot.getBotParms().autoGetOffBalanceStoneDistInches, 5);
//        opmode.sleep(500); // short pause to let things settle out
//
//        // 2) turn left toward the cipherbox - start heading is 0, final heading will be 90
//        robot.gyroDrive.hold(TURN_SPEED, 90.0, 4);
//
//        // 3) drive forward and center the bot in front of the indicated column
//        double lateralDriveDist;
//        if ( column == SamplePos.CRATER_NO_MARKER ) {
//            lateralDriveDist = robot.getBotParms().autoFarColumnLateralDistInches;
//        } else if ( column == SamplePos.DEPOT ) {
//            lateralDriveDist = robot.getBotParms().autoNearColumnLateralDistInches;
//        } else {
//            lateralDriveDist = robot.getBotParms().autoCenterColumnLateralDistInches;
//        }
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, lateralDriveDist, 3);
//
//        // 4) turn right to face the ciperbox - start heading is 90, final heading will be 0
//        robot.gyroDrive.hold(TURN_SPEED, 0.0, 4);
//
//        // score the glyph
//        // ??? TBD
//    }
//
//    public void scoreGlyphRedLeft(SamplePos column) {
//        //    1) drive forward off balancing stone (distance based on which balancing stone we are on)
//        double offStoneDriveDist;
//        if ( column == SamplePos.CRATER_NO_MARKER ) {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToFarDistInches;
//        } else if ( column == SamplePos.DEPOT ) {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToNearDistInches;
//        } else {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToCenterDistInches;
//        }
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, offStoneDriveDist, 3);
//        opmode.sleep(500); // short pause to let things settle out
//
//        //    2) turn right 90 degrees to face the glyph column (start heading 0, final heading -90)
//        robot.gyroDrive.hold(TURN_SPEED, -90.0, 4);
//
//        //    3) place glyph (***NOT DEFINED YET***)
//        // ??? TBD
//    }
//
//    public void scoreGlyphBlueLeft(SamplePos column) {
//        // 1) drive backward off the stone
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, -robot.getBotParms().autoGetOffBalanceStoneDistInches, 5);
//        opmode.sleep(500); // short pause to let things settle out
//
//        // 2) turn left toward the cipherbox - start heading is 0, final heading will be 90
//        robot.gyroDrive.hold(TURN_SPEED, 90.0, 4);
//
//        // 3) drive forward and center the bot in front of the indicated column
//        double lateralDriveDist;
//        if ( column == SamplePos.CRATER_NO_MARKER ) {
//            lateralDriveDist = robot.getBotParms().autoNearColumnLateralDistInches;
//        } else if ( column == SamplePos.DEPOT ) {
//            lateralDriveDist = robot.getBotParms().autoFarColumnLateralDistInches;
//        } else {
//            lateralDriveDist = robot.getBotParms().autoCenterColumnLateralDistInches;
//        }
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, lateralDriveDist, 3);
//
//        // 4) turn right to face the ciperbox - start heading is 90, final heading will be 0
//        robot.gyroDrive.hold(TURN_SPEED, 180.0, 4);
//
//        // score the glyph
//        // ??? TBD
//    }
//
//    public void scoreGlyphBlueRight(SamplePos column) {
//        //    1) drive BACKWARD off balancing stone (distance based on which balancing stone we are on)
//        double offStoneDriveDist;
//        if ( column == SamplePos.CRATER_NO_MARKER ) {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToFarDistInches;
//        } else if ( column == SamplePos.DEPOT ) {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToNearDistInches;
//        } else {
//            offStoneDriveDist = robot.getBotParms().autoGetOffBalancingStoneToCenterDistInches;
//        }
//        robot.encoderDrive.driveStraight(DRIVE_SPEED, -offStoneDriveDist, 3);
//        opmode.sleep(500); // short pause to let things settle out
//
//        //    2) turn right 90 degrees to face the glyph column (start heading 0, final heading -90)
//        robot.gyroDrive.hold(TURN_SPEED, -90.0, 4);
//
//        //    3) place glyph (***NOT DEFINED YET***)
//        // ??? TBD
    }
//
//}
