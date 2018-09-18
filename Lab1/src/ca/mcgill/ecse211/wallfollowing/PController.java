package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the p-type controller for Lab1 on the EV3 platform.
 * @author leaakkari
 * 
 *
 */
public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private final int SCALING_FACTOR = 2;
  private final int MIN_SPEED = 150;
  private final int MAX_SPEED = 350;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int error = 0;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    filterControl = 0;
  }
  /**
   * This method implements the error, the correction, and controls the robot speed
   */
  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    //Limiting the distance to 60 to avoid overflow 
    if(distance>60) {
	    	
	    distance = 60;
    }
    	//bandcenter = 25
	    error = (25 - distance);
	    
	    
	
	    if (Math.abs(error) <= bandWidth ) {// Within limits, same speed 
		   	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);// Start moving forward 
		   	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); 
		   	WallFollowingLab.leftMotor.forward(); 
		   	WallFollowingLab.rightMotor.forward();
	   	}
	    //if robot is too far away from wall
	    else if (error < 0){
	      //if too far away from the wall for too long, robot turns left 
	        turnLeft(Math.abs(error));
	      //if too close to the wall, robot turns right 
	    } else if(error > 0){
	      turnRight(Math.abs(error));
	      if(distance>2000) {
	      WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
	      WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
	      WallFollowingLab.rightMotor.backward();
	      WallFollowingLab.leftMotor.backward();
	      }
	    } else { //else Robot keeps going straight 
	      goStraight();
	    }
    
    
    
    
   
   
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

   
  }

  /**
   * This Method Turns Robot left
   * @param error
   */
  public void turnLeft(int error){
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - Math.abs(SCALING_FACTOR * error));
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + Math.abs(SCALING_FACTOR * error)-65);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  
  /**
   * This Method Turns Robot right
   * @param error
   */
  public void turnRight(int error){
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + Math.abs(SCALING_FACTOR * error+25));
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - Math.abs(SCALING_FACTOR * error) -65);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  
  /**
   * This method makes robot go straight
   */
  public void goStraight(){
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-20);
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-20);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
