package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.Range;

public class ShooterModel {
    public static class ShotSolution {
        public final double hoodPos;
        public final double velocityTicksPerSec;

        public ShotSolution(double hoodPos, double velocityTicksPerSec) {
            this.hoodPos = hoodPos;
            this.velocityTicksPerSec = velocityTicksPerSec;
        }
    }

    // Distance breakpoints (inches)
//    private static final double[] DIST = {45, 55, 65, 75, 85, 95, 105, 115, 125, 130};

    // Example values: replace with your tested data
//    private static final double[] HOOD = {0.93, 0.91, 0.89, 0.86, 0.83, 0.80, 0.78, 0.76, 0.75, 0.75};
//    private static final double[] VEL  = {1280, 1320, 1360, 1410, 1460, 1510, 1560, 1610, 1660, 1700};
    private static final double[] DIST = {47.638, 57.36, 69.72, 77.21, 91.18, 131.13, 136.7};

    // Example values: replace with your tested data
    private static final double[] HOOD = {0.93, 90, 0.88, 0.85, 0.80, 0.436, 0.421};
    private static final double[] VEL  = {1390, 1420, 1450, 1480, 1570, 2180, 2250};

    // Voltage compensation
    // V_NOM should match the battery voltage at which the VEL table was tuned.
    // V_NOM = the battery voltage at which your velocity table was tuned.
    // If the table was tuned on a FRESH battery, set V_NOM close to that voltage (e.g. 13.0).
    // When actual battery > V_NOM, velocity is REDUCED (prevents over-shooting on fresh battery).
    // When actual battery < V_NOM, velocity is INCREASED (compensates for sag).
    // Increase K_VOLT if fresh-battery over-shooting is still too large.
    private static final double V_NOM = 12.5;    // voltage at which table was calibrated (typical competition voltage)
    private static final double K_VOLT = 30.0;   // ticks/s per volt deviation (tune)

    // Limits
    private final double minHood;
    private final double maxHood;
    private final double minVel;
    private final double maxVel;

    public ShooterModel(double minHood, double maxHood, double minVel, double maxVel) {
        this.minHood = minHood;
        this.maxHood = maxHood;
        this.minVel = minVel;
        this.maxVel = maxVel;
    }

    public ShotSolution solve(double distanceInches, double batteryVoltage) {
        double hood = interp(DIST, HOOD, distanceInches);
        double vel = interp(DIST, VEL, distanceInches);

        // Add velocity when voltage sags
        vel += K_VOLT * (V_NOM - batteryVoltage);

        hood = Range.clip(hood, minHood, maxHood);
        vel = Range.clip(vel, minVel, maxVel);
        return new ShotSolution(hood, vel);
    }

    private static double interp(double[] x, double[] y, double xq) {
        if (xq <= x[0]) return y[0];
        if (xq >= x[x.length - 1]) return y[y.length - 1];

        for (int i = 0; i < x.length - 1; i++) {
            if (xq >= x[i] && xq <= x[i + 1]) {
                double t = (xq - x[i]) / (x[i + 1] - x[i]);
                return y[i] + t * (y[i + 1] - y[i]);
            }
        }
        return y[y.length - 1];
    }
}
