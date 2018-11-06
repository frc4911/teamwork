/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.robot.control.helpers.*;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//@Disabled
@Autonomous(name="Auto Sensor Test", group="Test")
public class AutoSensorTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTileRunner robot;

    @Override
    public void runOpMode() {

        telemetry.addData("STATUS", "Initializing robot, please wait...");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = TeamID.determineTeam(hardwareMap);
        robot.init(this, hardwareMap);

        // Send telemetry message to signify robot waiting;
        String teamID = String.format("Team %d:", robot.getBotParms().teamNumber);
        telemetry.addData(teamID, "Ready to run");    //
        telemetry.update();
        sleep(1000);

        // Do any setup required to be done before wait for start here

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(teamID, "WAIT FOR START -----");    //
        telemetry.update();
        waitForStart();

        robot.imu.start();
        robot.encoderDrive.resetEncoders();

        while ( opModeIsActive() ) {
            telemetry.addData("Team", robot.getBotParms().teamNumber);
            telemetry.addData(robot.encoderDrive.formatEncoders(), "");
            telemetry.addData(robot.imu.formatHeading(), "");
//            telemetry.addData(robot.colorSensor.formatColor(), "");
//            telemetry.addData(robot.distanceSensor.formatDistance(), "");
            telemetry.addData("Upper Arm Hard Limit", robot.upperArmLimit.isPressed());
//            telemetry.addData("Lower Arm Hard Limit", robot.lowerArmLimit.isPressed());
            telemetry.update();
            sleep(50);
        }

        telemetry.update();
        sleep(1000);
    }

}
