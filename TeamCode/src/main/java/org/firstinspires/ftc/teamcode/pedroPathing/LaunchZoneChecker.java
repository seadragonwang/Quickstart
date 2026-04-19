package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.CONSTANTS;

/**
 * Checks whether any wheel of the robot is inside one of the two launch zone triangles.
 *
 * Launch zones:
 *   Upper triangle: (0, 144), (72, 72), (144, 144)
 *   Lower triangle: (48, 0), (72, 24), (96, 0)
 */
public class LaunchZoneChecker {

    // Upper launch triangle vertices
    private static final double U_AX = 0,   U_AY = 144;
    private static final double U_BX = 72,  U_BY = 72;
    private static final double U_CX = 144, U_CY = 144;

    // Lower launch triangle vertices
    private static final double L_AX = 48,  L_AY = 0;
    private static final double L_BX = 72,  L_BY = 24;
    private static final double L_CX = 96,  L_CY = 0;

    /**
     * Check if any wheel of the robot is inside a launch zone.
     *
     * @param pose the robot's current pose (x, y, heading in radians)
     * @return true if at least one wheel is inside a launch zone
     */
    public static boolean isAnyWheelInLaunchZone(Pose pose) {
        double cx = pose.getX();
        double cy = pose.getY();
        double h = pose.getHeading();

        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        double halfW = CONSTANTS.ROBOT_HALF_WIDTH;
        double halfL = CONSTANTS.ROBOT_LENGTH_INCHES / 2.0;

        // 4 wheel offsets in robot-local frame: (forward, left)
        // LF = (+halfL, +halfW), RF = (+halfL, -halfW)
        // LR = (-halfL, +halfW), RR = (-halfL, -halfW)
        double[][] localOffsets = {
                { halfL,  halfW},  // left front
                { halfL, -halfW},  // right front
                {-halfL,  halfW},  // left rear
                {-halfL, -halfW},  // right rear
        };

        for (double[] off : localOffsets) {
            double wx = cx + off[0] * cosH - off[1] * sinH;
            double wy = cy + off[0] * sinH + off[1] * cosH;

            if (pointInTriangle(wx, wy, U_AX, U_AY, U_BX, U_BY, U_CX, U_CY)
                    || pointInTriangle(wx, wy, L_AX, L_AY, L_BX, L_BY, L_CX, L_CY)) {
                return true;
            }
        }
        return false;
    }

    /** Standard "same-side" triangle containment test using cross products. */
    private static boolean pointInTriangle(double px, double py,
                                           double ax, double ay,
                                           double bx, double by,
                                           double cx, double cy) {
        double d1 = sign(px, py, ax, ay, bx, by);
        double d2 = sign(px, py, bx, by, cx, cy);
        double d3 = sign(px, py, cx, cy, ax, ay);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    private static double sign(double px, double py,
                               double ax, double ay,
                               double bx, double by) {
        return (px - bx) * (ay - by) - (ax - bx) * (py - by);
    }
}

