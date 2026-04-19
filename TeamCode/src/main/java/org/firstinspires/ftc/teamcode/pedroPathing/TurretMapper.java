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
    private static final double[][] CALIBRATION = {
            {-110, 0.3301},
            {-105, 0.3382},
            {-100, 0.3416},
            { -95, 0.3473},
            { -93, 0.3510},
            { -90, 0.3575},
            { -85, 0.3685}, //0.3653
            { -80, 0.3785}, // 0.3750
            { -75, 0.3793}, //
            { -70, 0.3821},
            { -65, 0.3900},
            { -60, 0.3946},
            { -55, 0.4026},
            { -50, 0.4109},
            { -45, 0.4158},
            { -40, 0.4226},
            { -35, 0.4340},
            { -30, 0.4387},
            { -25, 0.4575}, // 0.4432
            { -20, 0.4685}, // 0.4502
            { -15, 0.4750}, // 0.4690
            { -10, 0.4810}, // 0.4748
            { -6, 0.4845}, // new
            { -5, 0.4900},
            {-3.7, 0.4955}, // new
            {   0, 0.5000},
            {   5, 0.5097},
            {  10, 0.5263}, // 0.5333
            {  15, 0.5372},
            {  20, 0.5440},
            {  25, 0.5564},
            {  30, 0.5692},
            {  35, 0.5708},
            {  40, 0.5829},
            {  45, 0.5890},
            {  50, 0.6113},
            {  55, 0.6128},
            {  60, 0.6170},
            {  65, 0.6260},
            {  70, 0.6325},
            {  75, 0.6385},
            {  80, 0.6442},
            {  85, 0.6527},
            {  90, 0.6632},
            {  95, 0.6660},
            { 100, 0.6699},
            { 105, 0.6767},
            { 110, 0.6858},
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

