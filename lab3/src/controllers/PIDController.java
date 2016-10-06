package controllers;

public class PIDController
{
    // CONTROL PARAMS
    float kp = 0.0f;
    float kd = 0.0f;
    float ki = 0.0f;

    // CONTROL VARIABLES
    float e_total = 0.0f;
    float e_last = 0.0f;

    public PIDController()
    {   
    }

    public PIDController(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void set_proportional(float k)
    {
        kp = k;
    }

    public void set_integral(float k)
    {
        ki = k;
    }

    public void set_derivative(float k)
    {
        kd = k;
    }

    public float step(double h, float error)
    {
        // Euler forward
        e_total += h * error;

        // Numerical differentiation
        float e_deriv = (float)((error - e_last) / h);

        // Calculate and apply input
        float correction = kp * error + ki * e_total + kd * e_deriv;

        // Update the derivative value
        e_last = error;
        
        return correction;
    }
}
