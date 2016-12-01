package tasks;

import common.BoardUtils.House;
import common.BoardUtils.Road;
import common.PIDController;
import common.RobotUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.robotics.navigation.Waypoint;

public class PizzaDropOffTask
{

    public static float HOUSE_DETECTION_EPSILON = 0.1f;
    public static int HOUSE_MIN_SPACING = 5;

    /**
     * Moves to the drop-off location and drops the pizza
     */
    public static boolean run_task(Navigator nav, RotateMoveController pilot, EV3UltrasonicSensor ultra,
            EV3ColorSensor color, NXTRegulatedMotor ultra_motor, NXTRegulatedMotor claw, House house)
    {
        // STEP 0: Init
        boolean success = true;

        PoseProvider ppv = nav.getPoseProvider();
        SampleProvider range_finder = ultra.getDistanceMode();

        SampleProvider average_range = new MeanFilter(range_finder, 3);
        float[] average_reading = new float[average_range.sampleSize()];

        PIDController controller = new PIDController(2.00f, 9.00f, 0.05f);

        // STEP 1: Turn the ultrasound to the correct side
        if (house.left)
        {
            ultra_motor.rotate(90);
        }
        else
        {
            ultra_motor.rotate(-90);
        }

        // STEP 2: Drive straight for the several centimeters to avoid counting
        // obstacles at the start of the road
        pilot.travel(Road.start_offset, true);

        while (pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                pilot.stop();

                break;
            }
        }

        // STEP 3: Drive straight and count houses as you go
        if (success)
        {
            pilot.travel(Road.start_offset, true);

            Pose last_house_pose = null;
            int house_count = 0;

            while (pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    pilot.stop();

                    break;
                }

                Pose current = ppv.getPose();

                // If we detect a house, add it to our counter
                average_range.fetchSample(average_reading, 0);
                if ((average_reading[0] - House.DIST_TO_ROAD / 100f) < HOUSE_DETECTION_EPSILON
                        && (last_house_pose == null || current.distanceTo(last_house_pose.getLocation()) > HOUSE_MIN_SPACING))
                {
                    house_count++;
                    last_house_pose = current;
                    
                    // If this house is the desired house, break from the loop
                    if (house_count == house.address)
                    {
                       break;
                    }
                }
            }
            
            // STEP 4: Turn and drop the pizza
            if (success)
            {
                if (house.left)
                {
                    pilot.rotate(-90);
                }
                else
                {
                    pilot.rotate(90);
                }
                
                success = success && RobotUtils.open_claw(claw);
            }
            
            // STEP 5: Return to the start of the road
            if (success)
            {
                nav.goTo(new Waypoint(house.road.start.getLocation()));
                
                while (nav.isMoving())
                {
                    // Break early condition
                    if (Button.ESCAPE.isDown())
                    {
                        success = false;
                        nav.stop();
                        break;
                    }
                }
            }
        }

        return success;
    }

    public static float COLOR_MIN_VALUE = 0.05f;
    public static float COLOR_MAX_VALUE = 0.33f;
    public static float COLOR_MID_VALUE = (COLOR_MAX_VALUE + COLOR_MIN_VALUE) / 2.0f;

    private static float get_error(float val)
    {
        return val - COLOR_MID_VALUE;
    }
}
