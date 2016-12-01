package testing;

import common.RangeFinderScan;
import common.RobotUtils;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import localization.DirectionKalmanPoseProvider;

public class ScannerTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = RobotUtils.LEFT_MOTOR;
        NXTRegulatedMotor right = RobotUtils.RIGHT_MOTOR;
        
        // Get the Ultrasound
        EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(RobotUtils.ULTRASOUND_PORT);
        SampleProvider rangefinder = ultra.getDistanceMode();

        // Get the gyro
        EV3GyroSensor gyro = new EV3GyroSensor(RobotUtils.GYRO_PORT);

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.get_track_width(), left,
                right);

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider ppv = new DirectionKalmanPoseProvider(pilot, gyro.getAngleMode());

        pilot.setTravelSpeed(15);
        pilot.setRotateSpeed(180 / 4);

        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }

        boolean success = true;
        RangeFinderScan scan = RangeFinderScan.scan(pilot, ppv, rangefinder, 160);
        success = success && (scan != null);
        
        if(success)
        {
            for (int x = 0; x < scan.scan_bandwidth; x++)
            {
                int scan_dist = (int)((3f - scan.range_spectrum[x][0])*30);
                for(int y = 0; y < scan_dist; y++)
                {
                    LCD.setPixel(x, y, 1);
                }
            }
        }
                
        Button.ESCAPE.waitForPressAndRelease();
    }
}
