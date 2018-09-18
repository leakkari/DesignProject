package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
/**
 * This class implemets the Bang Bang controller for Lab1 on the EV3 platform.
 * @author leaakkari
 *
 */

public class BangBangController implements UltrasonicController {

  //Given constants 
  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  
  //String constants to display on screen
  private final String RIGHT_TURN = "Turning right";
  private final String LEFT_TURN = "Turning left";
  private final String NO_TURN = "No turn required";
  private final String GO_BACK = "Reversing";
  private final String FAST_RIGHT_TURN = "Turning right quickly";
  private final String FAST_LEFT_TURN = "Turning left quickly";
  
  //Filter distances
  /*private final int FILTER_DISTANCE = 70;
  private final int FILTER_COUNT = 10;
  private final int ARRAY_LENGTH = 7;
  private int previousVals[] = new int[ARRAY_LENGTH];*/
  
  private String status;
  
 
  
  int walldistance = 20;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
   
    this.status = NO_TURN;
    
    
  }

  /**
   * This Method implements the error and controls the robot speed and movements.
   * 
   */
  @Override
  public void processUSData(int distance) {
    distance /= 1.3f;
    this.distance = distance;

  
  //calculates the error the robot needs to correct
    //bandcenyer here is 30.
  final int error = (int) 30 - distance; //33
  
  
  //If this occurs then the robot is at the correct distance from the wall and no changes need to be made
  if(bandwidth >= Math.abs(error)){
    setStatus(NO_TURN);
    setFast(); 
  } 
  else if( error > 0 ){ //if error is too far from the way, needs to turn right 
    setStatus(RIGHT_TURN);
    rightTurn();
    
   
  } else if (error < 0){ //if robot is too close to the wall turn left 
    setStatus(LEFT_TURN);
    leftTurn();
    }

}


  @Override
  public int readUSDistance() {
    return this.distance;
  }


//sets status provided in the parameter
private void setStatus(String stat){
  status = stat;
}


public String getStatus(){
  return status;
}

/**
 * This Method implements a left turn
 */
private void leftTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh-35); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorLow);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
}

/**
 * This method implements a right turn
 */
private void rightTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorLow - 55); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh-20);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
  
}

/**
 * This method makes the robot go backwards
 */
private void goBackwards(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh);
  WallFollowingLab.rightMotor.backward();
  WallFollowingLab.leftMotor.backward();
}

/**
 * This method makes the robot go forward
 */
private void setFast(){
  WallFollowingLab.leftMotor.setSpeed(motorLow+25);
  WallFollowingLab.rightMotor.setSpeed(motorLow+25); // Start robot moving forward
  WallFollowingLab.leftMotor.forward();
  WallFollowingLab.rightMotor.forward();
  
}

}
