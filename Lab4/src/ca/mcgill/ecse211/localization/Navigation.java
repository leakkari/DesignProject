package ca.mcgill.ecse211.localization;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.localization.Odometer;
import lejos.hardware.Button;

/**
 * 
 * @author Babettesmith
 *
 */

public class Navigation implements Runnable {

    //Constants and variables 
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	public static final double WHEEL_RAD = 2.2;
    public static final double TRACK = 12.9;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	private Odometer odometer;
	private OdometerData odoData;
	final static double CM_ERR = 0.4;
	final static double DEG_ERR = 0.6;
	final static int FAST = 80;
	final static int SLOW = 50;
	final static int ACCELERATION = 600;

	 public Navigation(Odometer odo) {
       this.odometer = odo;

       EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
       this.leftMotor = motors[0];
       this.rightMotor = motors[1];

       // set acceleration
       this.leftMotor.setAcceleration(ACCELERATION);
       this.rightMotor.setAcceleration(ACCELERATION);
   }

	/**
	 * Run method needed for use of threads 
	 */
	public void run() {

		// wait 5 seconds
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(300);
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
	}
	
	/**
	 * Method to get the robot to travel to set coordinates
	 * @param x
	 * @param y
	 */
	void travelTo(double x, double y) {
	 
	    //Gets current x,y and theta of the robot 
		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		
		//Workes out the change in x, y and theta and calcuates the distance needed to travel
		dx = x- currentX;
		dy = y - currentY;
		distanceToTravel = Math.sqrt(dx*dx+dy*dy);
		if(dy>=0) {
			dt=Math.atan(dx/dy);
		}
		else if(dy<=0&&dx>=0) {
			dt=Math.atan(dx/dy)+Math.PI;
		}
		else {
			dt=Math.atan(dx/dy)-Math.PI;
		}
		
		double differenceInTheta = (dt*180/Math.PI-currentT); 
		
		//Turns the robot in the necessary direction 
		turnTo(differenceInTheta); 
		
		//Drives robot forward the required distance 
	    leftMotor.setSpeed(FORWARD_SPEED-150);
	    rightMotor.setSpeed(FORWARD_SPEED-150);
	    leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), false);
	}
 
	/**
	 * Finds minimum angle and turns robot to it 
	 * @param theta
	 */
	void turnTo(double theta) {
		if(theta>180) {
			theta=360-theta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
		else if(theta<-180) {
			theta=360+theta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
		else {
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
		}
	}
	
	/**
	 * Converts distance to a value robot can use 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * Converts angle to value robot can use 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Gets the current x value from the odometer 
	 * @return
	 */
	public double getX() {
			return odometer.getXYT()[0];
		}

	/**
	 * Gets the current Y value from the odometer 
	 * @return
	 */
	public double getY() {
			return odometer.getXYT()[1];
		}

	/**
	 * Gets the current theta value from the odometer 
	 * @return
	 */
	public double getTheta() {
			return odometer.getXYT()[2];
	}

	/**
	 * Setter method for theta 
	 * @param theta
	 */
	public void setTheta(double theta) {
			odometer.setTheta(theta);
	}
		
	/**
	 * Setter method for x 
	 * @param x
	 */
	public void setX(double x) {
			odometer.setX(x);
	}
		
	/**
	 * Setter method for y
	 * @param y
	 */
	public void setY(double y) {
			odometer.setY(y);
	}

	/**
	 * Stops motors of the robot
	 */
	public void stopMotors() {
	      leftMotor.setSpeed(0);
	      rightMotor.setSpeed(0);
	   }
	    
}
