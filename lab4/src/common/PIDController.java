package common;

public class PIDController
{
    // CONTROL PARAMS
    float kp = 0.0f;
    float kd = 0.0f;
    float ki = 0.0f;

    // CONTROL VARIABLES
    float e_integ = 0.0f;
    float e_last = 0.0f;
    float e_deriv = 0.0f;
    
    float SAT_VAL = 0.10f;

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
    
    public float get_proportional()
    {
        return kp;
    }

    public float get_integral()
    {
        return ki;
    }

    public float get_derivative()
    {
        return kd;
    }
    
    public float get_derivative_error()
    {
        return e_deriv;
    }
    
    public float get_integral_error()
    {
        return e_integ;
    }
    
    public void zero_integral()
    {
        e_integ = 0.0f;
    }

    public float step(double h, float error)
    {
        // Euler forward
        e_integ += h * error;
        e_integ  = Math.min(e_integ, SAT_VAL);
        e_integ  = Math.max(e_integ, -SAT_VAL);

        // Numerical differentiation
        e_deriv = (float)((error - e_last) / h);

        //System.out.format("%2.2f %2.2f %2.2f%n", kp*error, ki * e_integ, kd * e_deriv);
        
        // Calculate and apply input
        float correction = kp * error + ki * e_integ + kd * e_deriv;

        // Update the derivative value
        e_last = error;
        
        return correction;
    }
}
