package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
@Config
public class VariableStorage {
    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(-90));

    public static int leftLinearSlidesPos = 0;
    public static int rightLinearSlidesPos = 0;
    public static double dumperPos = 0.17;
    public static Team currentTeam = Team.BLUE;
    public enum Team {
        BLUE, RED
    }
}
