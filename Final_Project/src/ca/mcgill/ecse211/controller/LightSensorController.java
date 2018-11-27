package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class implements the light sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class LightSensorController{
	
	private EV3ColorSensor lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	private float colorIntensity;
	private TextLCD lcd;
	
	/**
	 * This method is a constructor for this class
	 * @param lightSensor
	 * @param lcd
	 */
	public LightSensorController(EV3ColorSensor lightSensor, TextLCD lcd) {
		this.lightSensor = lightSensor;
		idColour = this.lightSensor.getRedMode();
		colorValue = new float[idColour.sampleSize()];
		this.lcd = lcd;
	}
	
	/**
	 * This method fetches samples from the light sensor 
	 * @return float color intensity
	 */
	public float fetch() {
		idColour.fetchSample(colorValue, 0);
		colorIntensity = colorValue[0];
		//lcd.drawString("Color: " + colorIntensity, 0, 5);
		return colorIntensity;
	}	
}
