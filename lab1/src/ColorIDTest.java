import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
public class ColorIDTest {
public static void main(String[] args) {
EV3ColorSensor color = new EV3ColorSensor(SensorPort.S4);
LCD.clear();
while (!Button.ENTER.isDown()) {
int sampleSize = color.sampleSize();
float[] idsample = new float[sampleSize];
color.getColorIDMode().fetchSample(idsample, 0);
LCD.clear();
System.out.println(idsample[0]);
}
color.close();
}
}
