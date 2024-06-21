package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private PIDCoefficients coefficients;
    private ElapsedTime time;
    private double clipMin, clipMax;
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean first = true;
    public PIDController(PIDCoefficients coeff, double mn, double mx) {
        coefficients = coeff;
        time = new ElapsedTime();
        clipMin = mn;
        clipMax = mx;
    }
    public double control(double error) {
        double currentTime = time.seconds();
        double cv = coefficients.p * error;

        if (!first) {
            double dt = currentTime - lastTime;
            sumError += error * dt;
            cv += coefficients.i * sumError + (error - lastError) / dt;
        }
        first = false;

        lastTime = currentTime;
        lastError = error;
        return Range.clip(cv, clipMin, clipMax);
    }

}
