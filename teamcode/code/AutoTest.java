package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.helpers.GameHelpers;
import org.firstinspires.ftc.teamcode.robot.control.helpers.HardwareTileRunner;
import org.firstinspires.ftc.teamcode.robot.control.helpers.TeamID;

public class AutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTileRunner robot;
    GameHelpers gameHelpers;

    @Override
    public void runOpMode() {

        telemetry.addData("STATUS", "Initializing robot, please wait...");
        telemetry.update();

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot = TeamID.determineTeam(hardwareMap);
        robot.init(this, hardwareMap);

        gameHelpers = new GameHelpers();
        gameHelpers.init(this, robot);

        // Send telemetry message to signify robot waiting;
        String teamID = String.format("Team %d:", robot.getBotParms().teamNumber);
        telemetry.addData(teamID, "Ready to run");    //
        telemetry.update();

        // Do any setup required to be done before wait for start here

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(teamID, "WAIT FOR START -----");    //
        telemetry.update();
        waitForStart();

        robot.imu.start();



        telemetry.update();
    }
}
