package org.firstinspires.ftc.teamcode.robot.control.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by thomp on 10/11/2017.
 */

public class TeamID {

    // Currently using the Rev IMU id to identify which team is running.
    // Other options include:
    //    - put a text file on each phone with the team number - *** MEH ***
    //    - hand change a variable value - *** YUCK DON'T WANT TO HAVE TO DO THIS ***
    //
    private static byte[] TEAM_5076_REV_IMU_ID = {   66, -96, -36, -101, 67, 84, 76, 81, 32, 32, 32, 51, 42, 51, 19, -1};
    private static byte[] TEAM_5423_REV_IMU_ID = {   36, -10, -69,  -17, 67, 84, 76, 81, 32, 32, 32, 52, 36, 25,  2, -1};
    private static byte[] TEAM_8985_REV_IMU_ID = { -100, 121,  74,  -27, 67, 84, 76, 81, 32, 32, 32, 51, 14, 15, 17, -1};
    private static byte[] TEAM_10232_REV_IMU_ID = { -8, -56, -118, 93, 80, 70, 77, 81, 32, 32, 32, 80, 47, 4, 9, -1};

    public static HardwareTileRunner determineTeam(HardwareMap ahwMap) {
        HardwareTileRunner robot;

        // The IMU on the Rev has a unique ID.  Use this to determine which team we running as.
        byte[] revID = RevIMU.getDeviceID(ahwMap);

        if ( revIDsMatch(revID, TEAM_5076_REV_IMU_ID) ) {
            robot = new Hardware5076();
        } else if ( revIDsMatch(revID, TEAM_5423_REV_IMU_ID) ) {
            robot = new Hardware5423();
        } else if ( revIDsMatch(revID, TEAM_8985_REV_IMU_ID) ) {
            robot = new Hardware8985();
        } else if ( revIDsMatch(revID, TEAM_10232_REV_IMU_ID) ) {
            robot = new Hardware10232();
        } else {
            String errMsg = String.format("TeamID.determineTeam(): unknown Rev IMU ID:%n");
            errMsg += "     " + RevIMU.formatDeviceID(ahwMap);
            throw new RuntimeException(errMsg);
        }

        return robot;
    }

    private static boolean revIDsMatch(byte[] id1, byte[] id2) {
        if ( id1.length != id2.length ) {
            return false;
        }

        for ( int i = 0 ; i < id1.length ; i++ ) {
            if ( id1[i] != id2[i] ) {
                return false;
            }
        }
        return true;
    }

}
