package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by thomp on 10/9/2017.
 */

public class Hardware5076 extends HardwareTileRunner {

    public Hardware5076() {}

    public void init(LinearOpMode opmode, HardwareMap ahwMap) {

        BotParams botParams = new BotParams();

        // Change botParms as needed for this robot ... defaults are provided in BotParams
        botParams.teamNumber = 5076;
        botParams.teamName = "CyberBeans";

        //Drive negation
        botParams.regularDriveFactor *= -1;
        botParams.extendJoystickFactor *= -1;

        //Speed scaling
        super.init(opmode, ahwMap, botParams);
    }
}
