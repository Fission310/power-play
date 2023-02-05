package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class AutoConstants {

    /** ==== CONSTANTS FOR CONE AUTOS ==== **/

    public static final double DELAY_PRELOAD_PICKUP = 4;
    public static final double DELAY_PICKUP = 2.6;
    public static final double DELAY_OPEN_CLAMP = 1.35;

    public static final double DELAY_SCORING = 0.15;

    public static final double[] SLIDE_EXTEND_POSITIONS = {12, 8.5, 6.4, 4, 1.2, 0, 0, 0};
    public static final double[] ARM_CONE_STACK_POSITIONS = {0.82, 0.82, 0.82, 0.82, 0.82, 0.82, 0.82};

    /** ==== END CONSTANTS FOR CONE AUTOS ==== **/

    /** DEFINITIONS OF PREFIXES (red / blue does not matter)
     *
     * RR - Red Right
     * RL - Red Left
     *
     * **/

    /** ======= CONSTANTS FOR RED RIGHT ======= **/

    public static final double RR_HEADING = Math.toRadians(180);
    public static final double RR_WALL_POS = -1 * (70.5 - (14 / 2.0));

    public static final double RR_CENTER_X = 35;
    public static final Pose2d RR_START_POSE = new Pose2d(RR_CENTER_X, RR_WALL_POS, RR_HEADING);

    public static final double RR_CONE_STACK_X = 60;
    public static final double RR_CONE_STACK_Y = -17.2; // -17.2

    public static final double RR_PRELOAD_CONE_STACK_Y = -17.45; // 16.5

    public static final double RR_HIGH_GOAL_X = 25.4;
    public static final double RR_HIGH_GOAL_Y = -7.5; // -8
    public static final double RR_HIGH_GOAL_Y_PRELOAD_OFFSET = 5.8 + RR_HIGH_GOAL_Y;

    public static final double RR_PRELOAD_X_OFFSET = 1;

    public static final double RR_HIGH_GOAL_ANGLE = 135;
    public static final double RR_HIGH_GOAL_TANGENT = 180;

    public static final double RR_CONE_STACK_ANGLE = 315;
    public static final double RR_CONE_STACK_END_ANGLE = 0;
    public static final double RR_CONE_STACK_ANGLE_OFFSET = 25;

    public static final double RR_PARK_LEFT_X = 9.3;

//    public static final Pose2d RR_PARK_LEFT = new Pose2d(RR_PARK_LEFT_X, RR_CONE_STACK_Y, RR_HEADING);
//    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(RR_CENTER_X, RR_CONE_STACK_Y, RR_HEADING);

    public static final Vector2d RR_HIGH_GOAL_VECTOR = new Vector2d(RR_HIGH_GOAL_X, RR_HIGH_GOAL_Y);
    public static final Vector2d RR_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_CONE_STACK_Y);
    public static final Vector2d RR_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_PRELOAD_CONE_STACK_Y);

    /** ======= END CONSTANTS FOR RED RIGHT ======= **/


    /** ======= CONSTANTS FOR RED LEFT ======= **/

    public static final double RL_HEADING = Math.toRadians(0);
    public static final double RL_WALL_POS = RR_WALL_POS;

    public static final double RL_CENTER_X = -RR_CENTER_X;
    public static final Pose2d RL_START_POSE = new Pose2d(RL_CENTER_X, RL_WALL_POS, RL_HEADING);

    public static final double RL_CONE_STACK_X = -RR_CONE_STACK_X;
    public static final double RL_CONE_STACK_Y = RR_CONE_STACK_Y;
    public static final double RL_PRELOAD_CONE_STACK_Y = RR_PRELOAD_CONE_STACK_Y;

    public static final double RL_HIGH_GOAL_X = -RR_HIGH_GOAL_X;
    public static final double RL_HIGH_GOAL_Y = RR_HIGH_GOAL_Y;
    public static final double RL_HIGH_GOAL_Y_PRELOAD_OFFSET = 6 + RL_HIGH_GOAL_Y;

    public static final double RL_PRELOAD_X_OFFSET = -RR_PRELOAD_X_OFFSET;

    public static final double RL_HIGH_GOAL_ANGLE = 180 - RR_HIGH_GOAL_ANGLE;
    public static final double RL_HIGH_GOAL_TANGENT = 180 - RR_HIGH_GOAL_TANGENT;

    public static final double RL_CONE_STACK_ANGLE = 180 - RR_CONE_STACK_ANGLE;
    public static final double RL_CONE_STACK_END_ANGLE = 180 - RR_CONE_STACK_END_ANGLE;
    public static final double RL_CONE_STACK_ANGLE_OFFSET = -RR_CONE_STACK_ANGLE_OFFSET;

    public static final double RL_PARK_LEFT_X = -RR_PARK_LEFT_X;

    public static final Pose2d RL_PARK_LEFT = new Pose2d(RL_PARK_LEFT_X, RL_CONE_STACK_Y, RL_HEADING);
    public static final Pose2d RL_PARK_MIDDLE = new Pose2d(RL_CENTER_X, RL_CONE_STACK_Y, RL_HEADING);

    public static final Vector2d RL_HIGH_GOAL_VECTOR = new Vector2d(RL_HIGH_GOAL_X, RL_HIGH_GOAL_Y);
    public static final Vector2d RL_CONE_STACK_VECTOR = new Vector2d(RL_CONE_STACK_X, RL_CONE_STACK_Y);
    public static final Vector2d RL_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RL_CONE_STACK_X, RL_PRELOAD_CONE_STACK_Y);

    /** ======= END CONSTANTS FOR RED LEFT ======= **/


    /** ====== CONSTANTS FOR PARK AUTOS ====== **/

    public static final double FORWARD_DIST = 25;
    public static final double LATERAL_DIST = 23;

    /** ====== END CONSTANTS FOR PARK AUTOS ====== **/
}
