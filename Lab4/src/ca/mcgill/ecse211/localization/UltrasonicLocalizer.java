	package ca.mcgill.ecse211.localization;

	import lejos.hardware.ev3.LocalEV3;
	import lejos.hardware.motor.EV3LargeRegulatedMotor;
	import lejos.hardware.port.Port;
	import lejos.hardware.sensor.EV3UltrasonicSensor;
	import lejos.hardware.sensor.SensorModes;
	import lejos.robotics.SampleProvider;

	/**
	 * Class for localization using the ultrasonic sensor 
	 * @author Babettesmith
	 *
	 */
	public class UltrasonicLocalizer implements Runnable {
		public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
		public enum RotationDirection {CW, CCW};
		private static final int FILTER_OUT = 30;
		public static final int DISTANCE_FROM_WALL = 40;
		public static int ROTATION_SPEED = 100;
		private EV3LargeRegulatedMotor leftMotor, rightMotor;
		private SampleProvider usSensor;
		private Odometer odometer;
		private float[] usData;	

		private int distance = 0;
		private int filter_control = 0;

		/**
		 * Constructor for UltrasonicLocalizerObject 
		 * @param leftMotor
		 * @param rightMotor
		 * @param TRACK
		 * @param WHEEL_RAD
		 * @throws OdometerExceptions
		 */
		public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK, double WHEEL_RAD) throws OdometerExceptions {
			odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			SensorModes us_sensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
			this.usSensor = us_sensor.getMode("Distance");
			this.usData = new float[usSensor.sampleSize()];
			this.leftMotor=leftMotor;
			this.rightMotor=rightMotor;}
		public void run() {
		// wait 5 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
					// there is nothing to be done here because it is not expected that
					// the odometer will be interrupted by another thread
				}
			if(localization.edge) {
				fallingedge();
				}
			else if(!localization.edge) {
				risingedge();
			}
			
	  }
		
		/**
		 * Method that will be called if falling edge localization is used (Robot is facing away from wall) 
		 * The robot looks for a wall on the left then rotates to look for wall on right, calculates 0 degree from these values  
		 */
		public void fallingedge(){
			double angle;
			
			//Gets information from sensor 
			fetch_sensor_value();
			
			//If robot is facing wall, it will start turning to look for the first wall it finds 
			if(this.distance<100){
				while(this.distance < 100) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				
				//Stops motors 
				leftMotor.stop(true);
				rightMotor.stop();
				
				while(this.distance > DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				
				leftMotor.stop(true);
				rightMotor.stop();
				odometer.setTheta(0);;

				while(this.distance < 100) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance > DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
				
			} else { //Robot is facing outwards 
				while(this.distance > DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				
				leftMotor.stop(true);
				rightMotor.stop();
				odometer.setTheta(0);

				while(this.distance < 100) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance > DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
			}
			
			//Using data from falling edge localization, robot rotates to the 0 degree angle and stops
			leftMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, 49+angle/2.0), true);
			rightMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, 49+angle/2.0), false);
			odometer.setTheta(0);
			leftMotor.stop(true);
			rightMotor.stop();
		}
		
		/**
		 * Method that will be called if rising edge localization is used (Robot is facing the wall) 
         * The robot looks for a wall on the left then rotates to look for wall on right, calculates 0 degree from these values
		 */
		public void risingedge(){
			double angle;
			for(int i=0; i <30; i++) {
				fetch_sensor_value();
			}//The robot is facing out
			if(this.distance>35){
				while(this.distance > 35) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance < DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();
				odometer.setTheta(0);

				while(this.distance > 35) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance < DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
					
			} else { //The robot is facing inwards 
				while(this.distance < DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();
				odometer.setTheta(0);

				while(this.distance > 35) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance < DISTANCE_FROM_WALL) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
			}
			
			//Using data from rising edge localization, robot rotates to the 0 degree angle and stops
			leftMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, angle/2.0-45), true);
			rightMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, angle/2.0-45), false);
			
			odometer.setTheta(0);
			leftMotor.stop(true);
			rightMotor.stop();
		}
			
		/**
		 * Gets sensor data from ultrasonic sensor 
		 */
		private void fetch_sensor_value() {
			usSensor.fetchSample(usData, 0); // acquire data
			int new_distance = (int) Math.abs(usData[0] * 100.0); // extract from buffer, cast to int
			// rudimentary filter - toss out invalid samples corresponding to null
			// signal.
			if (new_distance >= 255 && filter_control < FILTER_OUT) {
				filter_control++;
			} else if (new_distance >= 255) {
			this.distance = new_distance;
			} else {
			filter_control = 0;
			this.distance = new_distance;
			}
		}

		/**
		 * Converts inputted distance to values robot can use 
		 * @param radius
		 * @param distance
		 * @return
		 */
		public static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}

		/**
		 * Converts inputted values to angles robot can use 
		 * @param radius
		 * @param width
		 * @param angle
		 * @return
		 */
		public static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
	}
