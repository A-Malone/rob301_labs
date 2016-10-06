import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
public class ColorAmbientTest {
public static void main(String[] args) {
EV3ColorSensor color = new EV3ColorSensor(SensorPort.S4);
LCD.clear();
while (!Button.ENTER.isDown()) {
int sampleSize = color.sampleSize();
float[] ambsample = new float[sampleSize];
color.getAmbientMode().fetchSample(ambsample, 0);
LCD.clear();
System.out.println(ambsample[0]);
}
color.close();
}
}
