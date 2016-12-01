package testing;

import common.RangeFinderScan;
import common.Robot;
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

        // Create the robot with default parameters
        Robot robot = new Robot();

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro);
        robot.setPoseProvider(gyro_pose);
        
        robot.pilot.setTravelSpeed(15);
        robot.pilot.setRotateSpeed(180 / 4);
        

        Button.ENTER.waitForPressAndRelease();

        boolean success = true;
        RangeFinderScan scan = RangeFinderScan.scan(robot, 160);
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
        
        if (success)
        {   
            Button.ESCAPE.waitForPressAndRelease();
        }
    }
}
