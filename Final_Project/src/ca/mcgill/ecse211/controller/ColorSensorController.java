package ca.mcgill.ecse211.controller;

import java.text.DecimalFormat;
import java.util.Arrays;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

/** This class serves to detect and classify rings colors
 * It is the color sensor controller, so it controls the color sensor to detect the color of the rings
 * (blue, orange, red, yellow)
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class ColorSensorController {

	//private enum Color {BLUE, GREEN, YELLOW, ORANGE};

	private EV3ColorSensor colorSensor;
	// Light sensor objects
	private SampleProvider rgbValue;
	// Float arrays for Color data
	private float[] rgbData;
	
	// RGB Mean Values
//	private static final float[][] mean= {
//			{0.170390332f,0.767597595f,0.617868163f},
//			{0.402231815f,0.906190081f,0.13049561f},
//			{0.832694447f,0.538629888f,0.128443766f},
//			{0.953786617f,0.290982684f,0.074967764f}};
	
	
	private static final double[][]mean = {
			{0.1732410055,0.6778531281,0.7144947101},
			{0.4777487339,0.8592604804,0.1828320925},
			{0.8541708187,0.5005476676,0.140869603},
			{0.9547663589,0.2766071505,0.1091314998},
			{0.1345000000,0.0855000000,0.0122500000}};
	
	//Target color
	private static int targetColor = 4;
	/**
	 * This is a constructor for the ColorSensorController class
	 * @param colorSensor
	 */
	public ColorSensorController(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getRGBMode();
		rgbData = new float[rgbValue.sampleSize()];
	}

	
	/**
	 * This method allows to collect rgb values detected by the color sensor
	 * @return (array containing rgb values) 
	 */

	public float[] fetch() {
		rgbValue.fetchSample(rgbData, 0);
		return rgbData;
	}

	/**
//	 * This method allows to detect the color of the ring
//	 * 0-blue 1-green 2-yellow 3-orange 4-None
//	 * @return (integer representing the color)
//	 */
//	public int detect() {
//		int color;
//		do {
//			color = findMatch(fetch());
//			//   System.out.println("stuck in detect()");
//		} while (color == 4);
//		return color;
//	}
	
	public void beep() {
		switch(targetColor) {
		case 0: {
			Sound.beep();
			break;
		}
		case 1:{
			Sound.twoBeeps();
			break;
		}
		case 2:{
			Sound.beep();
			Sound.twoBeeps();
			break;
		}
		case 3:{
			Sound.twoBeeps();
			Sound.twoBeeps();
			break;
		}
		default: break;
		}
	}

	/**
	 * This method allows to match the readings and the mean to 
	 * determine the color detected
	 * @return (integer representing color)
	 */
	public int findMatch(float array[]) {
		
		double blue,green,yellow,orange,none;
		

		double euc = (Math.sqrt(array[0]*array[0] + array[1]*array[1] +array[2]*array[2]));
		
		// normalize
		double R=array[0]/euc;
		double G=array[1]/euc;
		double B=array[2]/euc;
		
		blue = Math.sqrt(Math.pow(R - mean[0][0], 2) + Math.pow(G - mean[0][1], 2) + Math.pow(B - mean[0][2], 2));
		green = Math.sqrt(Math.pow(R - mean[1][0], 2) + Math.pow(G - mean[1][1], 2) + Math.pow(B - mean[1][2], 2));
		yellow = Math.sqrt(Math.pow(R - mean[2][0], 2) + Math.pow(G - mean[2][1], 2) + Math.pow(B - mean[2][2], 2));
		orange = Math.sqrt(Math.pow(R - mean[3][0], 2) + Math.pow(G - mean[3][1], 2) + Math.pow(B - mean[3][2], 2));
		none = Math.sqrt(Math.pow(R - mean[4][0], 2) + Math.pow(G - mean[4][1], 2) + Math.pow(B - mean[4][2], 2));
		double[] list = {blue, green, yellow, orange, none};

		//sorted array
		Arrays.sort(list);
		
		if(list[0]== blue) {
			targetColor = 0;
			System.out.println("blue");
			return 0;
		}
		else if(list[0]== green) {
			targetColor = 1;
			System.out.println("green");
			return 1;
		}
		else if(list[0]== yellow) {
			targetColor = 2;
			System.out.println("yellow");
			return 2;
		}
		else if(list[0]== orange) {
			targetColor = 3;
			System.out.println("orange");
			return 3;
		}
		else {
			return 4;
		}
		
	}
}