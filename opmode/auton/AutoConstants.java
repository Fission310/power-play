package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class AutoConstants {

    /** ==== CONSTANTS FOR CONE AUTOS ==== **/

    public static final double DELAY_PRELOAD_PICKUP = 4.15; // 4.15
    public static final double DELAY_PICKUP = 2.42; // 2.52
    public static final double DELAY_OPEN_CLAMP = 0.9; // 1.35

    public static final double DELAY_SCORING = 0.05; // 0.15

    public static final double[] SLIDE_EXTEND_POSITIONS = {12, 8.8, 5.8, 3.3, 0.6, 0, 0, 0};
    public static final double[] ARM_CONE_STACK_POSITIONS = {0.83, 0.83, 0.84, 0.85, 0.86, 0.89, 0.92};

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

    public static final double RR_CONE_STACK_X = 59.8; // 60 for 5 cone!
    public static final double RR_CONE_STACK_Y = -17.2; // -17.2

    public static final double RR_PRELOAD_CONE_STACK_Y = -17;

    public static final double RR_HIGH_GOAL_X = 25.4;
    public static final double RR_HIGH_GOAL_Y = -7.08; // -7.08
    public static final double RR_HIGH_GOAL_Y_PRELOAD_OFFSET = 5.3 + RR_HIGH_GOAL_Y; // 5.3 +

    public static final double RR_PRELOAD_X_OFFSET = 1.5; // -0.3

    public static final double RR_HIGH_GOAL_ANGLE = 135;
    public static final double RR_HIGH_GOAL_TANGENT = 180;

    public static final double RR_CONE_STACK_ANGLE = 315;
    public static final double RR_CONE_STACK_END_ANGLE = 0;
    public static final double RR_CONE_STACK_ANGLE_OFFSET = 25;

    public static final double RR_PARK_LEFT_X = 11; // 9.3

//    public static final Pose2d RR_PARK_LEFT = new Pose2d(RR_PARK_LEFT_X, RR_CONE_STACK_Y, RR_HEADING);
//    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(RR_CENTER_X, RR_CONE_STACK_Y, RR_HEADING);

    public static final Vector2d RR_HIGH_GOAL_VECTOR = new Vector2d(RR_HIGH_GOAL_X, RR_HIGH_GOAL_Y);
    public static final Vector2d RR_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_CONE_STACK_Y);
    public static final Vector2d RR_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_PRELOAD_CONE_STACK_Y);


    public static final double RR_MED_GOAL_X = 25.4;
    public static final double RR_MED_GOAL_Y = -23.8;

    public static final Vector2d RR_MED_GOAL_VECTOR = new Vector2d(RR_MED_GOAL_X, RR_MED_GOAL_Y);

    // ODO CONSTANTS //
    public static final double RR_ODO_HIGH_GOAL_HEADING = Math.toRadians(135);
    public static final double RR_ODO_CONE_STACK_TANGENT = Math.toRadians(315);
    public static final double RR_ODO_CONE_STACK_HEADING = Math.toRadians(0);
    public static final double RR_ODO_MIDDLE_PARK_HEADING = Math.toRadians(295);
    public static final double RR_ODO_SIDE_PARK_HEADING = Math.toRadians(90);

    public static final double RR_ODO_CONE_STACK_X_SIX = 60.75; //61
    public static final double RR_ODO_CONE_STACK_Y_SIX = -11.8; // -12.1 or smth
    public static final Vector2d RR_ODO_CONE_STACK_VECTOR_SIX = new Vector2d(RR_ODO_CONE_STACK_X_SIX, RR_ODO_CONE_STACK_Y_SIX);
    public static final Pose2d RR_ODO_PRELOAD_CONE_STACK_POSE_SIX = new Pose2d(RR_CENTER_X, RR_ODO_CONE_STACK_Y_SIX, RR_HEADING);

    public static final double RR_ODO_CONE_STACK_X = 61.6; //61.5
    public static final double RR_ODO_CONE_STACK_Y = -11.5; //-12.6
    public static final Vector2d RR_ODO_CONE_STACK_VECTOR = new Vector2d(RR_ODO_CONE_STACK_X, RR_ODO_CONE_STACK_Y);
    public static final Pose2d RR_ODO_PRELOAD_CONE_STACK_POSE = new Pose2d(RR_CENTER_X, RR_ODO_CONE_STACK_Y, RR_HEADING);


    public static final double RR_ODO_HIGH_GOAL_X_SIX = 27.5; //28.1
    public static final double RR_ODO_HIGH_GOAL_Y_SIX = -3.35; //-3.8

    public static final double RR_ODO_PRELOAD_HIGH_GOAL_X_SIX = 29.2; //28.6
    public static final double RR_ODO_PRELOAD_HIGH_GOAL_Y_SIX = -0.5;

    public static final Vector2d RR_ODO_HIGH_GOAL_VECTOR_SIX = new Vector2d(RR_ODO_HIGH_GOAL_X_SIX, RR_ODO_HIGH_GOAL_Y_SIX);
    public static final Vector2d RR_ODO_PRELOAD_HIGH_GOAL_VECTOR_SIX = new Vector2d(RR_ODO_PRELOAD_HIGH_GOAL_X_SIX, RR_ODO_PRELOAD_HIGH_GOAL_Y_SIX);
    public static final Pose2d RR_ODO_PRELOAD_HIGH_GOAL_POSE_SIX = new Pose2d(RR_CENTER_X, RR_ODO_PRELOAD_HIGH_GOAL_Y_SIX, RR_HEADING);

    public static final double RR_ODO_HIGH_GOAL_X = 26.75; //26.75
    public static final double RR_ODO_HIGH_GOAL_Y = -2.1; //-2.69

    public static final double RR_ODO_PRELOAD_HIGH_GOAL_X = 28;
    public static final double RR_ODO_PRELOAD_HIGH_GOAL_Y = 0.2;

    public static final Vector2d RR_ODO_HIGH_GOAL_VECTOR = new Vector2d(RR_ODO_HIGH_GOAL_X, RR_ODO_HIGH_GOAL_Y);
    public static final Vector2d RR_ODO_PRELOAD_HIGH_GOAL_VECTOR = new Vector2d(RR_ODO_PRELOAD_HIGH_GOAL_X, RR_ODO_PRELOAD_HIGH_GOAL_Y);
    public static final Pose2d RR_ODO_PRELOAD_HIGH_GOAL_POSE = new Pose2d(RR_CENTER_X, RR_ODO_PRELOAD_HIGH_GOAL_Y, RR_HEADING);

    public static final Vector2d RR_ODO_MIDDLE_PARK_VECTOR = new Vector2d(RR_CENTER_X + 2.5, RR_ODO_CONE_STACK_Y + 0.5);

    public static final double RR_ODO_LEFT_PARK_X = 14.5;
    public static final Pose2d RR_ODO_LEFT_PARK_POSE = new Pose2d(RR_ODO_LEFT_PARK_X, RR_ODO_CONE_STACK_Y - 2.5, RR_ODO_SIDE_PARK_HEADING);

    public static final double RR_ODO_RIGHT_PARK_X = 59.5;
    public static final Pose2d RR_ODO_RIGHT_PARK_POSE = new Pose2d(RR_ODO_RIGHT_PARK_X, RR_ODO_CONE_STACK_Y - 2.5, RR_ODO_SIDE_PARK_HEADING);

    // ODO FOR MID GOAL //
    public static final double RR_ODO_MID_GOAL_HEADING = Math.toRadians(225);
    public static final double RR_ODO_MID_CONE_STACK_TANGENT = Math.toRadians(45);
    public static final double RR_ODO_MID_MIDDLE_PARK_HEADING = Math.toRadians(70); //0
    public static final double RR_ODO_MID_SIDE_PARK_HEADING = Math.toRadians(270);

    public static final double RR_ODO_MID_GOAL_X = 28.4; //29
    public static final double RR_ODO_MID_GOAL_Y = -19.7; //-19.24
    public static final double RR_ODO_PRELOAD_MID_GOAL_X = 28; //28
    public static final double RR_ODO_PRELOAD_MID_GOAL_Y = -24; //-24
    public static final Vector2d RR_ODO_MID_GOAL_VECTOR = new Vector2d(RR_ODO_MID_GOAL_X, RR_ODO_MID_GOAL_Y);
    public static final Vector2d RR_ODO_PRELOAD_MID_GOAL_VECTOR = new Vector2d(RR_ODO_PRELOAD_MID_GOAL_X, RR_ODO_PRELOAD_MID_GOAL_Y);
    public static final Pose2d RR_ODO_PRELOAD_MID_GOAL_POSE = new Pose2d(RR_CENTER_X, RR_ODO_PRELOAD_MID_GOAL_Y, RR_HEADING);

    public static final Vector2d RR_ODO_MID_MIDDLE_PARK_VECTOR = new Vector2d(RR_CENTER_X + 4, RR_ODO_CONE_STACK_Y + 3.25);

    public static final double RR_ODO_MID_CONE_STACK_X = 61.65; //61.5
    public static final double RR_ODO_MID_CONE_STACK_Y = -8; //-9.5
    public static final Vector2d RR_ODO_MID_CONE_STACK_VECTOR = new Vector2d(RR_ODO_MID_CONE_STACK_X, RR_ODO_MID_CONE_STACK_Y);

    public static final Pose2d RR_ODO_MID_LEFT_PARK_POSE = new Pose2d(RR_ODO_LEFT_PARK_X + 0.5, RR_ODO_CONE_STACK_Y - 2.5, RR_ODO_MID_SIDE_PARK_HEADING);
    public static final Pose2d RR_ODO_MID_RIGHT_PARK_POSE = new Pose2d(RR_ODO_RIGHT_PARK_X, RR_ODO_CONE_STACK_Y - 2.5, RR_ODO_MID_SIDE_PARK_HEADING);
    // END ODO FOR MID GOAL //

    public static final double C_HIGH_GOAL_X = 3;
    public static final double C_HIGH_GOAL_Y = -20;
    public static final Vector2d C_HIGH_GOAL_VECTOR = new Vector2d(C_HIGH_GOAL_X, C_HIGH_GOAL_Y);
    public static final Vector2d RR_ODO_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RR_CENTER_X, RR_ODO_CONE_STACK_Y);


    // END ODO CONSTANTS //

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
    public static final double RL_HIGH_GOAL_Y_PRELOAD_OFFSET = 5.4 + RL_HIGH_GOAL_Y;

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
