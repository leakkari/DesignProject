package ca.mcgill.ecse211.Navigation;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;

/**
 * This class implements the obstacle avoidance for the robot
 * @author leaakkari
 *
 */
public class ObstacleAvoidance implements Runnable {
	
	//motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	

	
	//maps array : implementing waypoints
	private double[][]  maps = new double[][]{
		{0*30.48,2*30.48}, 
		{1*30.48,1*30.48},
		{2*30.48,2*30.48},
		{2*30.48,1*30.48},
		{1*30.48,0*30.48}};
		
	//used to iterate through the maps array
	int i = 0;
	
	//Ultrasonic Sensor connected to port 2
	public static final Port usPort = LocalEV3.get().getPort("S2");
	
	//fetching data from the sensor
	public float[] usData;
	public SampleProvider usDistance ;
	
	//Robot speed
	public static final int FORWARD_SPEED = 250;
	public static final int ROTATE_SPEED = 150;
	
	//variables
	double dx, dy, dt;
	double travelDistance;
	
	
	private Odometer odometer;
	private OdometerData odoData;
	
	private final double TRACK;
	private final double WHEEL_RAD;
	
	
	// Constructor
	public ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions { 
		
		this.odometer = Odometer.getOdometer();
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		odoData = OdometerData.getOdometerData();
		
		//setting the coordinates
		odoData.setXYT(0 , 0 , 0);
		
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		
		//instance
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
		
		// usDistance provides samples from
		usDistance = usSensor.getMode("Distance"); 
		
		//buffer where data is returned
		this.usData = new float[usDistance.sampleSize()]; 
	}

	/**
	 * This method is the run method
	 * 
	 */
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
	 * This method is responsible for making the robot drive to the desired x and y coordinates
	 * @param x
	 * @param y
	 */
	void travelTo(double x, double y) {

		// dx = newx - nowx
		dx = x- odometer.getXYT()[0];
			
		// dy = newy - nowy
		dy = y - odometer.getXYT()[1];
			
		//Applying Pythagore to get the distance the robot has to travel
		travelDistance = Math.sqrt(dx*dx+dy*dy);
			
		// if robot going downwards
		if(dy<=0&&dx>=0) {
			dt=Math.atan(dx/dy)+Math.PI;
		}
		//if robot going upwards
		else if(dy>=0) {
			dt=Math.atan(dx/dy);
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
		rightMotor.rotate(convertDistance(WHEEL_RAD, travelDistance), true);
		
		
			
		// if distance between robot and obstacle <= 15
		// robot goes around obstacle
		while(isNavigating()) { 
			usDistance.fetchSample(usData,0);
			float distance = usData[0]*100;
			if(distance<= 15) {
				//checking robot position with respect to bounds to determine which way it will turn
					
				//check robot position, depending on position in the square, turn left
				if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
					
					//turn 90 degrees
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true); 
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					
					//travel 30.2 cm
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30.2), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30.2), false);
					
					//turn 90 degrees
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					
					//travel 40 cm
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
				}
				
				// or turn right
				else {
					
					//turn 90 degrees
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					
					//travel 30.2 cm
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30.2), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30.2), false);
					
					//turn 90 degrees
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					
					//travel 30.2 cm
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
				}
				
				// go back to waypoint we wanted to go to before robot had to avoid obstacle
				i--;
			}
		}
	}

	/**
	 * This method checks if the robot is navigating
	 * @return
	 */
	boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
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
	
	/**
	 * This method gets the minimal angle
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
}
