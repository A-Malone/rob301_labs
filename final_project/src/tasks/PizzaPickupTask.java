package tasks;

import common.BoardUtils.PizzaPedestal;
import common.RangeFinderScan;
import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;

public class PizzaPickupTask
{
    // ------------------------------------------------------------
    // ---- PIZZA PICKUP TASK
    // ------------------------------------------------------------

    // The initial width of the band to scan for the pizza
    private static int PIZZA_SCAN_BAND_WIDTH = 30;

    // The initial estimate of the pizza's distance
    private static float PIZZA_DISTANCE_ESTIMATE = 100;

    // Ideal distance to pizza for pickup
    private static float PIZZA_PICKUP_DISTANCE = 30;

    // Back-up distance to pickup pizza
    private static float PIZZA_BACKUP_DISTANCE = PIZZA_PICKUP_DISTANCE-11;

    // STEP 1: Zero in on pizza
    public static boolean move_to_pizza_task(Robot robot, PizzaPedestal pizza_pedestal)
    {
        // STEP 1.0: Init, and turn towards Pizza
        boolean success = true;

        PoseProvider ppv = robot.pose_provider;
        Pose current = ppv.getPose();

        float relative_heading = current.relativeBearing(pizza_pedestal.location);
        robot.pilot.rotate(relative_heading, true);

        while (robot.pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                robot.pilot.stop();
                break;
            }
        }

        // Resulting parameters
        float pizza_relative_heading = 0;
        float pizza_dist = 0;

        // Search parameters
        int search_width = PIZZA_SCAN_BAND_WIDTH;
        float last_estimated_distance = PIZZA_DISTANCE_ESTIMATE;

        while (success)
        {
            // STEP 1.1: Identify heading to pizza
            RangeFinderScan scan_results = RangeFinderScan.scan(robot, search_width);
            success = success && (scan_results != null);

            if (success)
            {
                // Linear scan to find measurements best matching pizza
                float min_error = Float.MAX_VALUE;

                float best_distance = 0;
                int best_index = -1;

                for (int i = 0; i < scan_results.range_spectrum.length; i++)
                {
                    if (Math.abs(scan_results.range_spectrum[i][0] - last_estimated_distance) < min_error)
                    {
                        best_distance = scan_results.range_spectrum[i][0];
                        best_index = i;
                    }
                }

                // Check if we found the pizza
                if (best_index != -1)
                {
                    pizza_relative_heading = scan_results.index_to_relative_heading(best_index);
                    pizza_dist = best_distance;
                    System.out.println("Pizza found " + pizza_relative_heading + ", " + ppv.getPose().getHeading());
                }
                else
                {
                    System.out.println("Pizza not found, exiting");
                    success = false;
                }
            }

            // STEP 1.2: Rotate towards pizza
            if (success)
            {
                robot.pilot.rotate(-pizza_relative_heading);
            }

            // STEP 1.2: Move towards pizza, checking to make sure we don't lose
            // alignment, or get too close.
            robot.pilot.travel(pizza_dist * 100 - PIZZA_PICKUP_DISTANCE, true);
            SampleProvider average_range = new MeanFilter(robot.ultra.getDistanceMode(), 3);

            while (robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    robot.pilot.stop();
                    break;
                }

                float est_goal_distance = last_estimated_distance - robot.pilot.getMovement().getDistanceTraveled();

                float[] sensor_reading = new float[average_range.sampleSize()];
                average_range.fetchSample(sensor_reading, 0);

                // If we're off by a factor of 2, re-locate pizza
                if (Math.abs(sensor_reading[0] - est_goal_distance) > est_goal_distance / 2)
                {
                    // We've lost lock-on on the pizza, re-acquire, and update
                    // our estimate of where it is
                    robot.pilot.stop();
                    last_estimated_distance = est_goal_distance;
                    break;
                }
            }
        }
        return success;
    }

    public static boolean dead_reckon_to_pizza_task(Robot robot, PizzaPedestal pizza_pedestal)
    {
        // STEP 1.0: Init, and turn towards Pizza
        boolean success = true;

        PoseProvider ppv = robot.pose_provider;
        Pose current = ppv.getPose();
        
        SampleProvider range_finder = robot.ultra.getDistanceMode();
        float[] sensor_reading = new float[range_finder.sampleSize()];

        float relative_heading = current.relativeBearing(pizza_pedestal.location);
        robot.pilot.rotate(relative_heading, true);

        while (robot.pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                robot.pilot.stop();
                break;
            }
        }

        current = ppv.getPose();
        robot.pilot.travel(current.distanceTo(pizza_pedestal.location) - PIZZA_PICKUP_DISTANCE, true);

        while (robot.pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                robot.pilot.stop();
                break;
            }
        }
        
        current = ppv.getPose();

        // SCAN FOR THE PIZZA STAND
        
        // Rotate to the left of the pizza stand
        robot.pilot.rotate(current.relativeBearing(pizza_pedestal.location) - PIZZA_SCAN_BAND_WIDTH / 2, true);

        while (robot.pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                robot.pilot.stop();
                break;
            }
        }

        // Sweep back to the right while scanning for the pedestal
        if (success)
        {
            robot.pilot.rotate(PIZZA_SCAN_BAND_WIDTH, true);
            double speed = robot.pilot.getRotateSpeed();
            robot.pilot.setRotateSpeed(180 / 10);

            while (robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    robot.pilot.stop();
                    break;
                }

                range_finder.fetchSample(sensor_reading, 0);
                float range = sensor_reading[0] * 100;

                LCD.drawString(String.valueOf(range), 0, 4);

                if (range > 0 && range < PIZZA_PICKUP_DISTANCE*2)
                {
                    robot.pilot.stop();
                    break;
                }
            }
            
            robot.pilot.setRotateSpeed(speed);
        }

        return success;
    }

    // STEP 1: Zero in on pizza
    public static boolean move_to_pizza_task_no_scan(Robot robot, PizzaPedestal pizza_pedestal)
    {
        // STEP 1.0: Init, and turn towards Pizza
        boolean success = true;

        SampleProvider range_finder = robot.ultra.getDistanceMode();
        SampleProvider average_range = new MeanFilter(robot.ultra.getDistanceMode(), 3);
        float[] sensor_reading = new float[range_finder.sampleSize()];

        PoseProvider ppv = robot.pose_provider;
        Pose current = ppv.getPose();

        // Resulting parameters
        float pizza_dist = 0;

        // Search parameters
        current = ppv.getPose();
        int search_width = PIZZA_SCAN_BAND_WIDTH;
        float last_estimated_distance = current.distanceTo(pizza_pedestal.location);

        robot.pilot.setRotateSpeed(180 / 6);

        while (success)
        {

            current = ppv.getPose();

            // Rotate to the left of the pizza stand
            robot.pilot.rotate(current.relativeBearing(pizza_pedestal.location) - search_width / 2, true);

            while (robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    robot.pilot.stop();
                    break;
                }
            }

            // Sweep back to the right while scanning for the pedestal
            if (success)
            {
                robot.pilot.rotate(search_width, true);

                while (robot.pilot.isMoving())
                {
                    // Break early condition
                    if (Button.ESCAPE.isDown())
                    {
                        success = false;
                        robot.pilot.stop();
                        break;
                    }

                    range_finder.fetchSample(sensor_reading, 0);
                    float range = sensor_reading[0] * 100;

                    LCD.drawString(String.valueOf(range), 0, 4);

                    if (range > last_estimated_distance - 40f && range < last_estimated_distance + 40f)
                    {
                        robot.pilot.stop();
                        pizza_dist = ppv.getPose().distanceTo(pizza_pedestal.location);
                        break;
                    }
                }
            }

            // STEP 1.2: Move towards pizza, checking to make sure we don't lose
            // alignment, or get too close.
            // System.out.println("Move to pizza");
            System.out.println("Moving to pizza");
            robot.pilot.travel(pizza_dist - PIZZA_PICKUP_DISTANCE, true);

            while (robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    robot.pilot.stop();
                    break;
                }

                float est_goal_distance = ppv.getPose().distanceTo(pizza_pedestal.location);

                average_range.fetchSample(sensor_reading, 0);

                // If we're off by a factor of 2, re-locate pizza
                if (Math.abs(sensor_reading[0] - est_goal_distance) > est_goal_distance / 2
                        && robot.pilot.getMovement().getDistanceTraveled() > 10)
                {
                    // We've lost lock-on on the pizza, re-acquire, and update
                    // our estimate of where it is
                    robot.pilot.stop();
                    last_estimated_distance = est_goal_distance;
                    break;
                }
            }
        }

        return success;
    }

    /**
     * Moves to the pizza from a starting location roughly oriented towards the
     * pizza, and picks it up using the claw arm.
     */
    public static boolean run_task(Robot robot, PizzaPedestal pizza_pedestal)
    {
        // STEP 0: Init
        boolean success = true;

        // STEP 1: Home in on pizza
        success = success && dead_reckon_to_pizza_task(robot, pizza_pedestal);

        // STEP 2: Turn around and position claw
        if (success)
        {
            robot.pilot.rotate(180);
            robot.open_claw();
        }

        // STEP 3: Back up to pizza
        if (success)
        {
            robot.pilot.travel(-PIZZA_BACKUP_DISTANCE);
        }

        // STEP 4: Close claw
        if (success)
        {
            robot.close_claw();
        }

        return success;
    }
}
