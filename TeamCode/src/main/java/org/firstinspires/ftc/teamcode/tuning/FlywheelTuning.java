package org.firstinspires.ftc.teamcode.tuning;

/** RPM and seconds throughout; outputs are normalized motor power. */
public final class FlywheelTuning {
    private FlywheelTuning() { }

    public static double filter(double previous, double measured, double dt) {
        return previous + (1 - Math.pow(.6, dt / .02)) * (measured - previous);
    }

    public static double[] fitFeedforward(double[] rpm, double[] power) {
        if (rpm.length != power.length || rpm.length < 3) throw new IllegalArgumentException("Need three plateaus");
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < rpm.length; i++) {
            if (!Double.isFinite(rpm[i]) || rpm[i] < 50 || !Double.isFinite(power[i]))
                throw new IllegalArgumentException("Invalid plateau");
            sx += rpm[i]; sy += power[i]; sxx += rpm[i] * rpm[i]; sxy += rpm[i] * power[i];
        }
        double denominator = rpm.length * sxx - sx * sx;
        if (denominator <= 1e-6) throw new IllegalArgumentException("Insufficient speed range");
        double kv = (rpm.length * sxy - sx * sy) / denominator;
        double ks = (sy - kv * sx) / rpm.length;
        if (kv <= 0 || ks < -0.03 || ks > 0.2) throw new IllegalArgumentException("Unexpected feedforward fit");
        if (ks < 0) { ks = 0; kv = sxy / sxx; }
        double residual = 0;
        for (int i = 0; i < rpm.length; i++) {
            double error = power[i] - ks - kv * rpm[i];
            residual += error * error;
        }
        if (Math.sqrt(residual / rpm.length) > .035) throw new IllegalArgumentException("Nonlinear or unstable plateaus");
        return new double[] {ks, kv};
    }

    public static final class Controller {
        public final double p, i, d, ks, kv;
        private double integral, previousRpm;
        private boolean initialized;
        public boolean saturated;

        public Controller(double p, double i, double d, double ks, double kv) {
            if (!Double.isFinite(p + i + d + ks + kv) || p < 0 || i < 0 || d < 0 || ks < 0 || kv < 0)
                throw new IllegalArgumentException("Invalid PID/feedforward gains");
            this.p = p; this.i = i; this.d = d; this.ks = ks; this.kv = kv;
        }

        public void reset() {
            integral = previousRpm = 0;
            initialized = saturated = false;
        }

        /** Derivative on measured speed avoids a target-step derivative kick. */
        public double update(double target, double rpm, double dt, double maxPower) {
            if (!Double.isFinite(target) || !Double.isFinite(rpm) || !Double.isFinite(dt)
                    || dt <= 0 || !Double.isFinite(maxPower) || maxPower <= 0 || maxPower > 1)
                throw new IllegalArgumentException("Invalid controller input");
            double derivative = initialized ? -(rpm - previousRpm) / dt : 0;
            previousRpm = rpm;
            initialized = true;
            double error = target - rpm;
            double proposedIntegral = integral + error * dt;
            double output = ks + kv * target + p * error + i * proposedIntegral + d * derivative;
            saturated = output < 0 || output > maxPower;
            if (!((output > maxPower && error > 0) || (output < 0 && error < 0))) integral = proposedIntegral;
            output = ks + kv * target + p * error + i * integral + d * derivative;
            return Math.max(0, Math.min(maxPower, output));
        }
    }
}
