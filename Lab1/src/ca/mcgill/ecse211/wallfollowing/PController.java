package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private final int SCALING_FACTOR = 10;
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

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    //caluclates error 
    this.error = (bandCenter - this.distance);
    
    //if robot is too far away from wall
    if (error < -bandWidth){
      filterControl++;
      //if too far away from the wall for too long, Beastie turns left 
      if (filterControl > FILTER_OUT){
        turnRight(Math.abs(error));
      }
      //if too close to the wall, Beastie turns right 
    } else if(error > bandWidth){
      filterControl = 0;
      turnLeft(Math.abs(error));
    } else { //else Beastsie keeps going straight 
      filterControl = 0;
      goStraight();
    }
    
    
    
   
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
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

    // TODO: process a movement based on the us distance passed in (P style)
  }

  //Turns Beastie left
  public void turnLeft(int error){
    WallFollowingLab.leftMotor.setSpeed(Math.max(MOTOR_SPEED - (SCALING_FACTOR * error), MIN_SPEED));
    WallFollowingLab.rightMotor.setSpeed(Math.max(MOTOR_SPEED + (SCALING_FACTOR * error), MAX_SPEED));
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  
  //Turns Beastie right
  public void turnRight(int error){
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + error);
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - error);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  
  //Keeps Beastie going straight
  public void goStraight(){
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.backward();
  }
  
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
