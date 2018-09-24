/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;


public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odoData;

	
	private float color[];
	 private static float csData;
	private double countx=0;
	private double county=0;
	private static final double size = 30.48;
	private int lines;
	private double bufferY;
	private int offset; //sensor is on the back of robot so this is the distance between sensor and front wheels.


	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {
		// There are 4 steps involved:
		// 1. Create a port object attached to a physical port (done already above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data

		this.odoData = Odometer.getOdometer();
		
		  
		    color = new float[Lab2.myColorSample.sampleSize()];
		    this.csData = color[0];
		    countx = 0.0;
		    county = 0.0;
		    bufferY=0.0;
		    offset = 13; //distance between front wheels and sensor
		    
		    
	
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		Lab2.myColorSample.fetchSample(color,0);
		while (true) {
			correctionStart = System.currentTimeMillis();
			Lab2.myColorSample.fetchSample(color, 0);
			csData = color[0];
			
			// encounters a black line
			// all numbers within if-condition correspond to range of angle (in degrees) on square corners
			if(csData<0.2) {
				
				//printing lines
				lines++;
				String print = "Lines Passed: " + lines;
				LCD.drawString(print, 0, 4);
				
				//beeping
				Sound.beep();
				
				
				if((odoData.getXYT()[2]>=350 && odoData.getXYT()[2] <=360)|| (odoData.getXYT()[2] >= 0.00 && odoData.getXYT()[2] <= 20)){ 
					// case 1: keep increasing y-position until wheels turn 90 degrees
					odoData.setY(county*size - offset); // robot is horizontally in line with origin	
					county++;
					
				}
				else if(odoData.getXYT()[2] >= 78 && odoData.getXYT()[2] <= 105){ // case 2: keep increasing x-position until 90 degree turn
					
					odoData.setX(countx*size - offset);
					countx++;
					
				}// robot is vertically in line with origin
				else if(odoData.getXYT()[2] >= 165 && odoData.getXYT()[2] <= 193){ // case 3: keep decreasing y-position until 90 degree turn
					county--;
					odoData.setY(county*size - offset);
					//bufferY = odoData.getXYT()[1];
					
				}	
				else if(odoData.getXYT()[2] >= 260 && odoData.getXYT()[2] <= 284){ // case 4: keep decreasing x-position until 90 degree turn
					countx--;
					odoData.setX(countx*size - offset);
					
				}	 
			}
		

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}


