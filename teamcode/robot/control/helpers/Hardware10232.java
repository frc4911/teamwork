package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware10232 extends HardwareTileRunner {

    public Hardware10232() {
    }

    public void init(LinearOpMode opmode, HardwareMap ahwMap) {

        BotParams botParams = new BotParams();

        // change botParams as needed for this robot ... defaults are provided in BotParams
        botParams.teamNumber = 10232;
        botParams.teamName = "Cyber";

        //botParams.countsPerMotorRev   = BotParams.NEVEREST_20_ENCODER_OUTPUT_COUNTS_PER_REV;

        //Drive negation
        botParams.regularDriveFactor *= -1;
        botParams.extendJoystickFactor *= -1;

        super.init(opmode, ahwMap, botParams);
    }
}
