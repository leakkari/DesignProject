package ca.mcgill.ecse211.obstacleAvoidance;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.obstacleAvoidance.*;
import ca.mcgill.ecse211.lab2.*;

public class Navigation {
	private Odometer odoData;
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	
	
	
	/**
	 * This  method  causes  the  robot  to  travel  to  the  absolute  field  location  (x,  y), specified  in tilepoints. 
	 * 
	 * This  method  should  continuously  call turnTo(double theta) and  then set  the motor speed to forward(straight). 
	 * This method will poll odometer for information 
	 *
	 * @param x
	 * @param y
	 */
	
	public void travelTo(double x, double y) {
		
	}
	
	/**
	 * This method causes the robot to turn (on point) to the absolute heading theta. 
	 * This method should turn a MINIMAL angle to its target
	 * @param theta
	 */
	
	//set minimal angle
	public void turnTo(double theta) {
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
	
	/**
	 * This  method  returns  true  if  another  thread  has  called travelTo()or turnTo() and  the method has yet to return; 
	 * false otherwise
	 * @return
	 */
	public boolean isNavigating() {
		return false;
		
	}
	
	

}
