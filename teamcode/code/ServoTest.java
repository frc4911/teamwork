package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.control.helpers.HardwareTileRunner;
import org.firstinspires.ftc.teamcode.robot.control.helpers.TeamID;

@TeleOp(name="Servo Test", group="Test")
public class ServoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTileRunner robot;

    @Override
    public void runOpMode() {
        double leftDrive, rightDrive, drive, turn, max;

        telemetry.addData("STATUS", "Initializing robot, please wait...");
        telemetry.update();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot = TeamID.determineTeam(hardwareMap);
        robot.init(this, hardwareMap);

        robot.marketTurretStow();

        // Send telemetry message to signify robot waiting;
        String teamID = String.format("Team %d:", robot.getBotParms().teamNumber);
        telemetry.addData(teamID, "Ready to run");
        telemetry.update();

        // Do any setup required to be done before wait for start here

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(teamID, "WAIT FOR START -----");
        telemetry.update();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            boolean boost = gamepad1.right_bumper;
            robot.driveBothSidesArcade(drive,turn,boost);

            if (gamepad2.a) {
                robot.markerTurretIn();
            } else if (gamepad2.b) {
                robot.markerTurretOut();
            }

            if (gamepad2.left_trigger > 0.2) {
                robot.openLatch();
            } else if (gamepad2.right_trigger > 0.2) {
                robot.closeLatch();
            }

            if (!opModeIsActive()) {
                return;
            }
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
