package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutoConstants {

    /** ==== CONSTANTS FOR CONE AUTOS ==== **/

    public static final double DELAY_SCORE = 0.5;
    public static final double DELAY_PICKUP = 0.5;

    /** ==== END CONSTANTS FOR CONE AUTOS ==== **/

    /** DEFINITIONS OF PREFIXES
     *
     * RR - Red Right
     * RL - Red Left
     *
     *
     * BR - Blue Right
     * BL - Blue Left
     *
     * **/

    /** ======= CONSTANTS FOR RED RIGHT ======= **/

    public static final double RR_HEADING = Math.toRadians(180);
    public static final double RR_WALL_POS = -1 * (70.5 - (14 / 2.0));

    public static final double RR_CENTER_X = 35;
    public static final Pose2d RR_START_POSE = new Pose2d(RR_CENTER_X, RR_WALL_POS, RR_HEADING);

    public static final double RR_CONE_STACK_X = 60;
    public static final double RR_CONE_STACK_Y = -18;
    public static final double RR_PRELOAD_CONE_STACK_Y = -16;

    public static final double RR_HIGH_GOAL_X = 22.75;
    public static final double RR_HIGH_GOAL_Y = -6;

    public static final double RR_PRELOAD_X_OFFSET = 0.5;

    public static final double RR_HIGH_GOAL_ANGLE = 140;
    public static final double RR_HIGH_GOAL_TANGENT = 180;

    public static final double RR_CONE_STACK_ANGLE = 320;
    public static final double RR_CONE_STACK_END_ANGLE = 0;
    public static final double RR_CONE_STACK_ANGLE_OFFSET = 20;

    public static final double RR_PARK_LEFT_X = 10;

    public static final Pose2d RR_PARK_LEFT = new Pose2d(RR_PARK_LEFT_X, RR_CONE_STACK_Y, RR_HEADING);
    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(RR_CENTER_X, RR_CONE_STACK_Y, RR_HEADING);

    public static final Vector2d RR_HIGH_GOAL_VECTOR = new Vector2d(RR_HIGH_GOAL_X, RR_HIGH_GOAL_Y);
    public static final Vector2d RR_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_CONE_STACK_Y);

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

    /** ======= END CONSTANTS FOR RED LEFT ======= **/


    /** ======= CONSTANTS FOR BLUE RIGHT ======= **/

    public static final double BR_HEADING = RL_HEADING;
    public static final double BR_WALL_POS = -RL_WALL_POS;

    public static final double BR_CENTER_X = RL_CENTER_X;
    public static final Pose2d BR_START_POSE = new Pose2d(BR_CENTER_X, BR_WALL_POS, BR_HEADING);

    public static final double BR_CONE_STACK_X = RL_CONE_STACK_X;
    public static final double BR_CONE_STACK_Y = -RL_CONE_STACK_Y;
    public static final double BR_PRELOAD_CONE_STACK_Y = -RR_PRELOAD_CONE_STACK_Y;

    public static final double BR_HIGH_GOAL_X = RL_HIGH_GOAL_X;
    public static final double BR_HIGH_GOAL_Y = -RL_HIGH_GOAL_Y;

    public static final double BR_PRELOAD_X_OFFSET = RL_PRELOAD_X_OFFSET;

    public static final double BR_HIGH_GOAL_ANGLE = -RL_HIGH_GOAL_ANGLE;
    public static final double BR_HIGH_GOAL_TANGENT = RL_HIGH_GOAL_TANGENT;

    public static final double BR_CONE_STACK_ANGLE = -RL_CONE_STACK_ANGLE;
    public static final double BR_CONE_STACK_END_ANGLE = RL_CONE_STACK_END_ANGLE;
    public static final double BR_CONE_STACK_ANGLE_OFFSET = -RL_CONE_STACK_ANGLE_OFFSET;

    public static final double BR_PARK_LEFT_X = RL_PARK_LEFT_X;

    public static final Pose2d BR_PARK_LEFT = new Pose2d(BR_PARK_LEFT_X, BR_CONE_STACK_Y, BR_HEADING);
    public static final Pose2d BR_PARK_MIDDLE = new Pose2d(BR_CENTER_X, BR_CONE_STACK_Y, BR_HEADING);

    public static final Vector2d BR_HIGH_GOAL_VECTOR = new Vector2d(BR_HIGH_GOAL_X, BR_HIGH_GOAL_Y);
    public static final Vector2d BR_CONE_STACK_VECTOR = new Vector2d(BR_CONE_STACK_X, BR_CONE_STACK_Y);

    /** ======= END CONSTANTS FOR BLUE RIGHT ======= **/


    /** ======= CONSTANTS FOR BLUE LEFT ======= **/

    public static final double BL_HEADING = RR_HEADING;
    public static final double BL_WALL_POS = -RR_WALL_POS;

    public static final double BL_CENTER_X = RR_CENTER_X;
    public static final Pose2d BL_START_POSE = new Pose2d(BL_CENTER_X, BL_WALL_POS, BL_HEADING);

    public static final double BL_CONE_STACK_X = RR_CONE_STACK_X;
    public static final double BL_CONE_STACK_Y = -RR_CONE_STACK_Y;
    public static final double BL_PRELOAD_CONE_STACK_Y = -RR_PRELOAD_CONE_STACK_Y;

    public static final double BL_HIGH_GOAL_X = RR_HIGH_GOAL_X;
    public static final double BL_HIGH_GOAL_Y = -RR_HIGH_GOAL_Y;

    public static final double BL_PRELOAD_X_OFFSET = RR_PRELOAD_X_OFFSET;

    public static final double BL_HIGH_GOAL_ANGLE = -RR_HIGH_GOAL_ANGLE;
    public static final double BL_HIGH_GOAL_TANGENT = -RR_HIGH_GOAL_TANGENT;

    public static final double BL_CONE_STACK_ANGLE = -RR_CONE_STACK_ANGLE;
    public static final double BL_CONE_STACK_END_ANGLE = 360 + RR_CONE_STACK_END_ANGLE;
    public static final double BL_CONE_STACK_ANGLE_OFFSET = -RR_CONE_STACK_ANGLE_OFFSET;

    public static final double BL_PARK_LEFT_X = RR_PARK_LEFT_X;

    public static final Pose2d BL_PARK_LEFT = new Pose2d(BL_PARK_LEFT_X, BL_CONE_STACK_Y, BL_HEADING);
    public static final Pose2d BL_PARK_MIDDLE = new Pose2d(BL_CENTER_X, BL_CONE_STACK_Y, BL_HEADING);

    public static final Vector2d BL_HIGH_GOAL_VECTOR = new Vector2d(BL_HIGH_GOAL_X, BL_HIGH_GOAL_Y);
    public static final Vector2d BL_CONE_STACK_VECTOR = new Vector2d(BL_CONE_STACK_X, BL_CONE_STACK_Y);

    /** ======= END CONSTANTS FOR BLUE LEFT ======= **/


    /** ====== CONSTANTS FOR PARK AUTOS ====== **/

    public static final double FORWARD_DIST = 25;
    public static final double LATERAL_DIST = 23;

    /** ====== END CONSTANTS FOR PARK AUTOS ====== **/
}
