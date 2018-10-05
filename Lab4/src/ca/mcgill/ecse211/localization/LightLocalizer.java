package ca.mcgill.ecse211.localization;


import ca.mcgill.ecse211.localization.UltrasonicLocalizer.RotationDirection;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * 
 * Implements localization for the robot using the light sensor. 
 * Assumes the robot is placed in the bottom left corner on the diagonal 
 * axis and it has a good idea of what its heading is already.
 *
 */
public class LightLocalizer implements Runnable{

	private SampleProvider color_sample_provider;
	private float[] color_samples;
	private float light_value;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int d=13;
	private double TRACK;
	private double WHEEL_RAD;

	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK, double WHEEL_RAD){
		this.leftMotor = leftMotor; 
		this.rightMotor = rightMotor;
		EV3ColorSensor colour_sensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		color_sample_provider = colour_sensor.getMode("Red");
		color_samples = new float[colour_sensor.sampleSize()];
		this.TRACK=TRACK;
		this.WHEEL_RAD=WHEEL_RAD;
	}
	public void run() {
		// wait 5 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
					// there is nothing to be done here because it is not expected that
					// the odometer will be interrupted by another thread
				}			
			do_localization();
	 }
	public void do_localization(){
		fetch_sensor_value();
		while(light_value>0.3){
			leftMotor.forward();
			rightMotor.forward();
			fetch_sensor_value();
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		fetch_sensor_value();
		while(light_value>0.3){
			leftMotor.forward();
			rightMotor.forward();
			fetch_sensor_value();
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	public void fetch_sensor_value(){
		color_sample_provider.fetchSample(color_samples,0);
		this.light_value=color_samples[0];
	}

	public static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}

	public static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
}