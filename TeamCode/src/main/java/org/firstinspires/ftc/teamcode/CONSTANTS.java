package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class CONSTANTS {
    public final static double DRIVE_POWER = 0.8;
    public final static double CLOSE_INTAKE_POWER = 0.9;
    public final static double JR_OUTTAKE_BLOCK = 0.78;
    public final static double JR_OUTTAKE_OPEN = 0.0;
    public final static int FAR_OUTTAKE_VELOCITY = 2200; // tip of far triangle: 2200, back part 2300
    public final static int CLOSE_OUTTAKE_VELOCITY = 1600;
    public final static double HOOD_MAX_POS = 0.92;
    public final static double HOOD_MIN_POS = 0.2;

    public final static double kP = 2.6/*1.25*/;
    public final static double kI = 0.08;
    public final static double kD = 0.0;

    // Robot dimensions (inches) — 240mm x 240mm //0.57 99.1 108.3
    public static final double ROBOT_WIDTH_INCHES = 15.7;
    public static final double ROBOT_LENGTH_INCHES = 15.2;
    public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH_INCHES / 2.0;
    public static final double RED_GOAL_POSITION_X = 139; // 144 - 8
    public static final double RED_GOAL_POSITION_Y = 139; // 144 - 8
    public final static double BLUE_GOAL_POSITION_X = 1;
    public final static double BLUE_GOAL_POSITION_Y = 141.5;
    // Turret offset from robot center (in inches, robot-local frame)
    // TURRET_OFFSET_FORWARD: positive = turret is in front of robot center
    // TURRET_OFFSET_LEFT: positive = turret is to the left of robot center
    // Measure from your robot center to the turret pivot and adjust these values
    public static final double TURRET_OFFSET_FORWARD = -1.4; // inches behind robot center (adjust!)
    public static final double TURRET_OFFSET_LEFT = 0.0;     // inches left of robot center (adjust!)

    // Maximum turret rotation in degrees (left or right from center)
    public static final double MAX_TURRET_ANGLE = 135;
    // Global turret trim: applied to all directions. positive = shift left, negative = shift right.
    public static double TURRET_ANGLE_OFFSET_DEG = 0.0;
    // Extra trim applied ONLY when turret turns right (errorDegrees < 0).
    // If turret overshoots right only when turning right, increase this (positive = correct left).
    public static double TURRET_ANGLE_OFFSET_RIGHT_DEG = 0.0;
    // Scale factor applied to errorDegrees before lookup. Values < 1.0 reduce overshoot on both sides.
    // If turret overshoots both left AND right, reduce this below 1.0 (e.g. 0.85).
    public static double TURRET_ANGLE_SCALE = 0.85;
    public static final double LIMELIGHT_APRIL_TAG_POS = 0.42;
    public static final double LIMELIGHT_BALL_POS = 0.53;
    public static final double TURRET_POSITION_PER_DEGREE =0.0016798245614035089;

    public  static  final Pose BLUE_NEAR_TELE_START = new Pose(36, 70, Math.toRadians(180));
    public  static  final Pose RED_NEAR_TELE_START = new Pose(108, 70, Math.toRadians(0));
    public  static  final Pose BLUE_FAR_TELE_START = new Pose(44, 25, Math.toRadians(180));
    public static final Pose RED_FAR_TELE_START = new Pose(100, 25, Math.toRadians(0));
}
