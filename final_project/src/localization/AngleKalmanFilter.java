package localization;

import lejos.hardware.Button;
import lejos.utility.KalmanFilter;
import lejos.utility.Matrix;

/**
 * Implementation of a Kalman filter using the Matrix class
 */
public class AngleKalmanFilter
{
    private double a, b, q, mu, muBar, sigma, sigmaBar;
    private Matrix c, i, r, ct;
    private Matrix gain, s;

    public AngleKalmanFilter(double a, double b, Matrix c, double q, Matrix r)
    {
        this.a = a;
        this.b = b;
        this.c = c;
        this.q = q;
        this.r = r;
        this.ct = c.transpose();
    }

    public void setState(double mean, double covariance)
    {
        this.mu = mean;
        this.sigma = covariance;
        this.i = Matrix.identity(2, 2);
    }

    public void update(double control, Matrix measurement)
    {   
        // Control update step 1: calculate the predicted mean
        muBar = mu*a + b*control;

        // Control update step 2: calculate the predicted covariance
        sigmaBar = sigma*a*a + q;

        // Calculate the Kalman Gain
        s = c.times(sigmaBar).times(ct).plus(r);
        gain = ct.times(sigmaBar).times(s.inverse());
        
        // Measurement update: calculate the new mean
        mu = muBar + gain.times(measurement.minus(c.times(muBar))).get(0,0);
        
        // Calculate the new covariance
        sigma = 1 - gain.times(c).times(sigmaBar).get(0, 0);
    }

    public double getMean()
    {
        return mu;
    }

    public double getCovariance()
    {
        return sigma;
    }

    public double getPredictedMean()
    {
        return muBar;
    }

    public double getPredictedCovariance()
    {
        return sigmaBar;
    }

    public Matrix getGain()
    {
        return gain;
    }
}
