package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.control.helpers.HardwareTileRunner;
import org.firstinspires.ftc.teamcode.robot.control.helpers.TeamID;

@TeleOp(name="Home Arm", group="Test")
public class HomeArm extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTileRunner robot;

    @Override
    public void runOpMode() {
        double leftDrive, rightDrive, drive, turn, max;

        boolean setEncodersNow = false;

        telemetry.addData("STATUS", "Initializing robot, please wait...");
        telemetry.update();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot = TeamID.determineTeam(hardwareMap);
        robot.init(this, hardwareMap);

        robot.setExtendLimits(false);

        // Send telemetry message to signify robot waiting;
        String teamID = String.format("Team %d:", robot.getBotParms().teamNumber);
        telemetry.addData(teamID, "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(teamID, "WAIT FOR START -----");
        telemetry.update();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            //Button actions
            if (gamepad2.a) {
                setEncodersNow = true;
                robot.setArmPower(-0.2);
            } else if (gamepad2.b) {
                setEncodersNow = true;
                robot.setArmPower(0.2);
            } else if (setEncodersNow) {
                setEncodersNow = false;
                robot.setArmPower(0);
                robot.armReset();
            }
//            if (gamepad2.left_trigger > 0.2) {
//                robot.openLatch();
//            } else if (gamepad2.right_trigger > 0.2) {
//                robot.closeLatch();
//            }
//            if (gamepad2.left_bumper) {
//                robot.markerTurretIn();
//            } else if (gamepad2.right_bumper) {
//                robot.markerTurretOut();
//            }

            telemetry.addData("Arm Position", robot.getExtendCurrentPosition());
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
