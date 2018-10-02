package ca.mcgill.ecse211.Navigation;
import lejos.hardware.sensor.*;
/**
 * This class implements the navigation for the robot
 */
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;

public class Navigation implements Runnable {
	
	//motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//maps array
	private double[][]  wayPoints = new double[][]{
		{0*30.48,2*30.48}, 
		  {1*30.48,1*30.48},
		  {2*30.48,2*30.48},
		  {2*30.48,1*30.48},
		  {1*30.48,0*30.48}};
	
	//variables
	double dx, dy, dt;
	double travelDistance;
		  
	private final double TRACK;
	private final double WHEEL_RAD;
	
	//Robot speed
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	
	private Odometer odometer;
	private OdometerData odoData;
	
	//Constructor
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
	    this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    odoData = OdometerData.getOdometerData();
	    odoData.setXYT(0 , 0 , 0);
	    this.TRACK = TRACK;
	    this.WHEEL_RAD = WHEEL_RAD;

	}

	//Method for thread
	public void run() {

		// wait 5 seconds
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(300);
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		// implemented this for loop so that navigation will work for any number of points
		for (int i = 0; i < wayPoints.length; i++) { 
			travelTo(wayPoints[i][0], wayPoints[i][1]);
		}
	}

	void travelTo(double x, double y) {
		 //dx
		dx = x- odometer.getXYT()[0];
		
		//dy
		dy = y - odometer.getXYT()[1];;
		
		//Applying Pythagore to get the distance the robot has to travel
		travelDistance = Math.sqrt(dx*dx+dy*dy);
		
		
		if(dy>=0) {
			dt=Math.atan(dx/dy);
		}
		else if(dy<=0&&dx>=0) {
			dt=Math.atan(dx/dy)+Math.PI;
		}
		else {
			dt=Math.atan(dx/dy)-Math.PI;
		}
		
		// dtheta = how much the robot should turn
		double dTheta = (dt*180/Math.PI-odometer.getXYT()[2]); 
		
		//turn minimal angle
		turnTo(dTheta); 
		
		// drive robot forward
	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.rotate(convertDistance(WHEEL_RAD, travelDistance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, travelDistance), false);
	}
	
	/**
	 * This Method gets the minimal angle
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
	 * This method checks if the robot is navigating
	 * @return
	 */    
	boolean isNavigating() {
	 if((leftMotor.isMoving() && rightMotor.isMoving()))
		 return true;
	 else 
		 return false;

	}
	
	/**
	 * This method converts the distance
	 * @param radius
	 * @param distance
	 * @return
	 */
	 private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	 
	 /**
		 * This method converts the angle from  radian to degrees
		 * @param radius
		 * @param width
		 * @param angle
		 * @return
		 */
	 private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
