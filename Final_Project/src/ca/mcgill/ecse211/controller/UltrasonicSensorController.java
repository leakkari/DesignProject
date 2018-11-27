package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class implements the ultrasonic sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class UltrasonicSensorController{

	private SensorModes usSensor;
	private SampleProvider usDistance;
	//private SampleProvider average;
	private float[] usData;

	private int distance = 0;
	
	//private boolean isRunning;

	private TextLCD lcd;

	/**
	 * This is a constructor for this class
	 * @param usSensor
	 * @param lcd
	 */
	public UltrasonicSensorController(SensorModes usSensor, TextLCD lcd) {
		this.usSensor = usSensor;
		usDistance = usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];
		this.lcd = lcd;
		//isRunning = true;
	}

	/**
	 * This method fetches samples from the ultrasonic sensor 
	 * @return integer distance from sensor to object
	 */
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		
//		if (distance > 50) {
//			distance = 255;
//		}
		//lcd.clear();
		//lcd.drawString("Distance: " + distance, 0, 5);
		return distance;
	}
	
	// public void setRunning(boolean state) {
	// 	isRunning = state;
	// }
//	public void run() {
//		while (isRunning) {
//
//			usDistance.fetchSample(usData, 0); // acquire data
//			distance = (int)(usData[0] * 100.0); // extract from buffer, cast to int
//			//cont.process(distance); // now take action depending on value
//			
//			try {
//				Thread.sleep(15);
//			} catch (Exception e) {
//			} // Poor man's timed sampling
//		}
//	}

}
