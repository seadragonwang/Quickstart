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
            {-110, 0.1764},
            {-105, 0.1911},
            {-100, 0.2058},
            { -95, 0.2205},
            { -90, 0.2352},
            { -85, 0.2499},
            { -80, 0.2646},
            { -75, 0.2793},
            { -70, 0.2940},
            { -65, 0.3088},
            { -60, 0.3235},
            { -55, 0.3382},
            { -50, 0.3529},
            { -45, 0.3676},
            { -40, 0.3823},
            { -35, 0.3971},
            { -30, 0.4118},
            { -25, 0.4265},
            { -20, 0.4412},
            { -15, 0.4559},
            { -10, 0.4706},
            {  -5, 0.4853},
            {   0, 0.5000},
            {   5, 0.5147},
            {  10, 0.5294},
            {  15, 0.5441},
            {  20, 0.5588},
            {  25, 0.5735},
            {  30, 0.5882},
            {  35, 0.6029},
            {  40, 0.6176},
            {  45, 0.6323},
            {  50, 0.6470},
            {  55, 0.6617},
            {  60, 0.6764},
            {  65, 0.6911},
            {  70, 0.7058},
            {  75, 0.7205},
            {  80, 0.7352},
            {  85, 0.7499},
            {  90, 0.7646},
            {  95, 0.7793},
            { 100, 0.7940},
            { 105, 0.8087},
            { 110, 0.8234},
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
