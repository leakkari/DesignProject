package ca.mcgill.ecse211.localization;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.localization.Odometer;
import lejos.hardware.Button;

public class Navigation implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	private Odometer odometer;
	private OdometerData odoData;
	private double[][]  wayPoints = new double[][]{{0*30.48,2*30.48}, // change values for different maps
												  {1*30.48,1*30.48},
												  {2*30.48,2*30.48},
												  {2*30.48,1*30.48},
												  {1*30.48,0*30.48}};
												 //array list for points
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

	// run method (required for Thread)
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
		// implemented this for loop so that navigation will work for any number of points
		for (int i = 0; i < wayPoints.length; i++) { 
			travelTo(wayPoints[i][0], wayPoints[i][1]);
		}
	}

	void travelTo(double x, double y) {
		currentX = odometer.getXYT()[0];//get the current position of the robot
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		
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
		}//Mathematical convention
		
		// initial angle is 0||2pi, same direction as y-axis, going clockwise
		double differenceInTheta = (dt*180/Math.PI-currentT); // robot has to turn "differenceInTheta",
		//turn the robot to the desired direction
		turnTo(differenceInTheta); 
		
		// drive forward required distance
	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), false);
	}
 
	void turnTo(double theta) {
		if(theta>180) {//angel convention, turn in correct minimal angle
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
	
	    
	boolean isNavigating() {
	 if((leftMotor.isMoving() && rightMotor.isMoving()))
		 return true;
	 else 
		 return false;

	}
	
	 private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	 private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

		public double getX() {
			return odometer.getXYT()[0];
		}

		public double getY() {
			return odometer.getXYT()[1];
		}

		public double getTheta() {
			return odometer.getXYT()[2];
		}

		public void setTheta(double theta) {
			odometer.setTheta(theta);
		}
		
		public void setX(double x) {
			odometer.setX(x);
		}
		
		public void setY(double y) {
			odometer.setY(y);
		}
}
