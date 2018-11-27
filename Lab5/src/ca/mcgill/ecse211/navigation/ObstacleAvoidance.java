/**
 *@author leaakkari
 */
package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidance extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//Ultrasonic Sensor connected to port 2
	//public static final Port usPort = LocalEV3.get().getPort("S2");
	
	//fetching data from the sensor
	public float[] usData;
	public SampleProvider us ;
	

	private double deltax;
	private double deltay;

	// current location of the vehicle
	private double currx;
	private double curry;
	private double currTheta;

	// set constants
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 80;
	private static final double TILE_SIZE = 30.48;
	
	int i=0;

	//maps array : implementing waypoints
		private double[][]  maps = new double[][]{
			{1*30.48,6*30.48}, 
			{2*30.48,6*30.48},
			{2*30.48,1*30.48},
			{3*30.48,1*30.48},
			{3*30.48,6*30.48}};
			
	
	

	// constructor for navigation
	public ObstacleAvoidance(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider us) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.us = us;
		this.usData = new float[us.sampleSize()];
		
	}

	// main run method for navigation
	public void run() {


		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(300);
		}
		
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			
		}
		//iterating through the maps array to travel to x and y coordinates
		while(i < maps.length) { 
			travelTo(maps[i][0], maps[i][1]);
			i++;
			
		}



	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	private void travelTo(double x, double y) {

		currx = odometer.getXYT()[0];
		curry = odometer.getXYT()[1];

		deltax = x - currx;
		deltay = y - curry;

		// Calculate the angle to turn around
		currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltax, deltay) - currTheta;

		double hypot = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
		
		/**
		 * Avoid obstacles by rotating around them
		 */
		while(isNavigating()) { 
			us.fetchSample(usData,0);
			float distance = usData[0]*100;
			if(distance<= 7) {
				
			// **FOR DEBUGGING**
				
				//print distance
				LCD.drawInt((int) distance, 5, 5);
				//beep 
				Sound.beepSequence();
				
			//** *** *** *** *** *** *** 
				
				//checking robot position with respect to bounds to determine which way it will turn
					
				//check robot position, depending on position in the square, turn left
				if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
					
					//turn 90 degrees
					leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true); 
					rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
					
					//turn 90 degrees
					leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
					rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
					
					//turn 90 degrees
					leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);  
					rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
				}
				
				// or turn right
				else {
					
					//turn 90 degrees
					leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);  
					rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
					
					//turn 90 degrees
					leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
					rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
					
					//turn 90 degrees
					leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true); 
					rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);
					
					//travel 20 cm
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
					
					
				}
				
				//go back to last waypoint
				i--;
							
			}
				
		}

	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	private void turnTo(double theta) {

		// ensures minimum angle for turning
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// rotate motors at set speed

		// if angle is negative, turn to the left
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * If one of the motors is moving, the robot is navigating
	 * @return
	 */
	boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
	}
		

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
		

}
