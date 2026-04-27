package org.firstinspires.ftc.teamcode.pedroPathing;

/**
 * Maps turret angle (degrees) to servo position using calibrated lookup table
 * with linear interpolation between data points.
 *
 * Convention: negative degrees = turret right (servo < 0.5),
 *             positive degrees = turret left (servo > 0.5)
 */
public class TurretMapper {

    // Calibrated data points: [degrees, servoPos]
    // Sorted from most-right (-110°) to most-left (+110°)
    // Interpolated from anchors: (-90, 0.7884), (0, 0.5000), (90, 0.2315)
    // Slope -90→0:  -0.003204/deg   Slope 0→90: -0.002983/deg
    private static final double[][] CALIBRATION = {
            {-110, 0.8525},
            {-105, 0.8365},
            {-100, 0.8204},
            { -95, 0.8044},
            { -90, 0.7884},
            { -85, 0.7724},
            { -80, 0.7564},
            { -75, 0.7403},
            { -70, 0.7243},
            { -65, 0.7083},
            { -60, 0.6923},
            { -55, 0.6762},
            { -50, 0.6602},
            { -45, 0.6442},
            { -40, 0.6282},
            { -35, 0.6122},
            { -30, 0.5961},
            { -25, 0.5801},
            { -20, 0.5641},
            { -15, 0.5481},
            { -10, 0.5320},
            {  -5, 0.5160},
            {   0, 0.5000},
            {   5, 0.4851},
            {  10, 0.4702},
            {  15, 0.4552},
            {  20, 0.4403},
            {  25, 0.4254},
            {  30, 0.4105},
            {  35, 0.3956},
            {  40, 0.3807},
            {  45, 0.3657},
            {  50, 0.3508},
            {  55, 0.3359},
            {  60, 0.3210},
            {  65, 0.3061},
            {  70, 0.2912},
            {  75, 0.2763},
            {  80, 0.2613},
            {  85, 0.2464},
            {  90, 0.2315},
            {  95, 0.2166},
            { 100, 0.2017},
            { 105, 0.1867},
            { 110, 0.1718},
    };

    /**
     * Convert a turret angle in degrees to a servo position using
     * lookup table with linear interpolation.
     *
     * @param degrees turret angle: negative = left, positive = right
     * @return servo position (0.0 – 1.0)
     */
    public static double degreesToServoPos(double degrees) {
        // Clamp to calibrated range
        if (degrees <= CALIBRATION[0][0]) return CALIBRATION[0][1];
        if (degrees >= CALIBRATION[CALIBRATION.length - 1][0]) return CALIBRATION[CALIBRATION.length - 1][1];

        // Find the two surrounding data points and interpolate
        for (int i = 0; i < CALIBRATION.length - 1; i++) {
            double deg0 = CALIBRATION[i][0];
            double deg1 = CALIBRATION[i + 1][0];
            if (degrees >= deg0 && degrees <= deg1) {
                double t = (degrees - deg0) / (deg1 - deg0);
                double pos0 = CALIBRATION[i][1];
                double pos1 = CALIBRATION[i + 1][1];
                return pos0 + t * (pos1 - pos0);
            }
        }

        // Fallback (should never reach here)
        return 0.5;
    }
}
