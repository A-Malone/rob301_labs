import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
public class GyroTest {
public static void main(String[] args) {
EV3GyroSensor tilt = new EV3GyroSensor(SensorPort.S3);
LCD.clear();
while (!Button.ENTER.isDown()) {
int sampleSize = tilt.sampleSize();
float[] tiltsample = new float[sampleSize];
float[] ratesample = new float[sampleSize];
tilt.getAngleMode().fetchSample(tiltsample, 0);
tilt.getRateMode().fetchSample(ratesample, 0);
LCD.clear();
System.out.println(tiltsample[0] + " " + ratesample[0]);
}
tilt.close();
}
}