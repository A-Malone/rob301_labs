package common;

import java.util.Arrays;
import java.util.List;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;

public class Tasks
{
    // ---- PIZZA PICKUP TASK
    private static int SCAN_BAND_WIDTH = 10; // The width of the band to scan
                                             // for the pizza
    private static float PIZZA_GOAL_DISTANCE = 10; // Ideal distance to pizza
                                                   // for pickup task

    // Pickup the pizza using a proportional controller
    // Starting assumptions: Robot is oriented +-SCAN_BAND deg to pizza, within
    // ultrasonic sensor range
    public static boolean pizza_pickup_task(Navigator nav, DifferentialPilot pilot, EV3UltrasonicSensor ultra,
            Motor claw)
    {
        // STEP 0: Init
        boolean success = true;

        PoseProvider ppv = nav.getPoseProvider();
        SampleProvider range_finder = ultra.getDistanceMode();

        
        // STEP 1: Zero in on pizza
        float pizza_relative_heading = 0;
        float pizza_dist = 0;
        
        while (success)
        {
            // STEP 1.1: Identify heading to pizza
            RangeFinderScan scan_results = RangeFinderScan.scan(pilot, ppv, range_finder, SCAN_BAND_WIDTH);
            success = success && (scan_results != null);

            if (success)
            {
                // Linear scan to find minimum distance
                float min_distance = Float.MAX_VALUE;
                int min_index = -1;

                for (int i = 0; i < scan_results.range_spectrum.length; i++)
                {
                    if (scan_results.range_spectrum[i][0] < min_distance)
                    {
                        min_index = i;
                    }
                }

                // Check if we found the pizza
                if (min_index != -1)
                {
                    pizza_relative_heading = scan_results.index_to_relative_heading(min_index);
                    pizza_dist = min_distance;
                } else
                {
                    success = false;
                }
            }

            // STEP 1.2: Rotate towards pizza
            if (success)
            {
                pilot.rotate(pizza_relative_heading);
            }

            // STEP 1.2: Move towards pizza, checking to make sure we don't lose alignement, or get too close.
            pilot.travel(pizza_dist * 100 - PIZZA_GOAL_DISTANCE, true);

            while (pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    break;
                }

                Delay.msDelay(20);
            }
        }

        return success;
    }
}
