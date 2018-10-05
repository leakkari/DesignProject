	package ca.mcgill.ecse211.localization;

	import lejos.hardware.ev3.LocalEV3;
	import lejos.hardware.motor.EV3LargeRegulatedMotor;
	import lejos.hardware.port.Port;
	import lejos.hardware.sensor.EV3UltrasonicSensor;
	import lejos.hardware.sensor.SensorModes;
	import lejos.robotics.SampleProvider;

	public class UltrasonicLocalizer implements Runnable {
		public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
		public enum RotationDirection {CW, CCW};
		private static final int FILTER_OUT = 30;
		public static final int WALL_DISTANCE = 40;
		public static int ROTATION_SPEED = 100;
		private EV3LargeRegulatedMotor leftMotor, rightMotor;
		private SampleProvider usSensor;
		private Odometer odometer;
		private float[] usData;	

		private int distance = 0;
		private int filter_control = 0;

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
		public void fallingedge(){
			//facing the wall at the very beginning
			double angle;
			fetch_sensor_value();
			if(this.distance<100){
				while(this.distance < 100) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.backward();
					rightMotor.forward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				while(this.distance > WALL_DISTANCE) {
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

				while(this.distance > WALL_DISTANCE) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
			}else{//facing out
				while(this.distance > WALL_DISTANCE) {
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

				while(this.distance > WALL_DISTANCE) {
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
			leftMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, 49+angle/2.0), true);
			rightMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, 49+angle/2.0), false);
			odometer.setTheta(0);
			leftMotor.stop(true);
			rightMotor.stop();
		}
		public void risingedge(){
			double angle;
			for(int i=0; i <30; i++) {
				fetch_sensor_value();
			}//facing out
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

				while(this.distance < WALL_DISTANCE) {
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

				while(this.distance < WALL_DISTANCE) {
					leftMotor.setSpeed(ROTATION_SPEED); 
					rightMotor.setSpeed(ROTATION_SPEED);
					leftMotor.forward();
					rightMotor.backward();

					fetch_sensor_value();
				}
				leftMotor.stop(true);
				rightMotor.stop();

				angle=odometer.getXYT()[2];
					//facing in
			}else{
				while(this.distance < WALL_DISTANCE) {
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

				while(this.distance < WALL_DISTANCE) {
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
			leftMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, angle/2.0-45), true);
			rightMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, angle/2.0-45), false);
			
			odometer.setTheta(0);
			leftMotor.stop(true);
			rightMotor.stop();
		}
			
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

		public static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}

		public static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
	}
