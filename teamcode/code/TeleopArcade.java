/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.control.helpers.*;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic POV Drive TeleopArcade for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Arcade", group="Teleop")
//
// @Disabled
public class TeleopArcade extends LinearOpMode {

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

        float extendPower = 0;

        robot.latchStow();

        // Send telemetry message to signify robot waiting;
        String teamID = String.format("Team %d:", robot.getBotParms().teamNumber);
        telemetry.addData(teamID, "Ready to run");
        telemetry.update();

        // Do any setup required to be done before wait for start here
        robot.setExtendLimits(true);

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
            extendPower = gamepad2.left_stick_y;
            boolean boost = gamepad1.right_bumper;

            // Output the safe vales to the motor drives.
            robot.driveBothSidesArcade(drive,turn,boost);
            robot.teleopArmExtend(extendPower);

            if (gamepad2.left_trigger > 0.2) {
                robot.openLatch();
            } else if (gamepad2.right_trigger > 0.2) {
                robot.closeLatch();
            }

            // Send telemetry message to signify robot running
//            telemetry.addData("climb position (starts at 0)", robot.getExtendCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {
                return;
            }
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
