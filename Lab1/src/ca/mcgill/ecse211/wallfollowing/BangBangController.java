package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

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
  private final int FILTER_DISTANCE = 70;
  private final int FILTER_COUNT = 10;
  private final int ARRAY_LENGTH = 7;
  private int previousVals[] = new int[ARRAY_LENGTH];
  
  private String status;
  
  private int filterCount = 0;
  
  float actualDistance = 0;
  


  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    this.status = NO_TURN;
    
    //for loop checking the values of the previous vals array
    for (int i = 0; i < previousVals.length; i++){
      previousVals[i] = 0;
    }
  }

  @Override
  public void processUSData(int distance) {
    distance /= 1.3f;
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    if ((distance >= FILTER_DISTANCE && this.filterCount < FILTER_COUNT) || distance < 0 ){
      this.filterCount++;
    } else if (distance > 0 || distance >= FILTER_DISTANCE){ //resets sensor distance depending on which case is true 
      this.filterCount = 0;
      actualDistance = (float)(getAverageDist(distance));
    } 
  
  if(actualDistance <= 0) {
    return;
  }
  
  //calculates the error the robot needs to correct
  final int error = (int) actualDistance - bandCenter;
  
  
    if (status.equals(RIGHT_TURN)){
    if (actualDistance < 15){
      setStatus(FAST_RIGHT_TURN);
      fastRightTurn();
      return;
    } else if(actualDistance >= 15 && actualDistance < 30){
      setStatus(RIGHT_TURN);
      rightTurn();
      return;
    }
  }
  
  //Reverses if the distance is less than 7 
  if (actualDistance < 7) {
    setStatus(GO_BACK);
    goBackwards();
    return;
  }
  
  //If this occurs then the robot is at the correct distance from the wall and no changes need to be made
  if(bandwidth > Math.abs(error)){
    setStatus(NO_TURN);
    setFast();
    
  } //else if(error >= 15){ // if the robot is too far from the wall and need urgent re-adjustment
    //setStatus(FAST_LEFT_TURN);
    //fastLeftTurn();
  //}
  else if( error > 0 ){ //if error is too far from the way, needs to turn left 
    setStatus(LEFT_TURN);
    leftTurn();
    if(bandwidth > Math.abs(error)){
      setStatus(NO_TURN);
      setFast();
    }
  } else if (error < 0){ //if robot is too close to the wall 
    setStatus(RIGHT_TURN);
    rightTurn();
    if(bandwidth > Math.abs(error)){
      setStatus(NO_TURN);
      setFast();
    }
  }
}


  @Override
  public int readUSDistance() {
    return this.distance;
  }

//method to compute the average to use in resetting the filter distance
public int getAverageDist(int dist){
  int j = 0;
  int sum = 0; 
  for (int i = 0; i < previousVals.length -1; i++){
    if(previousVals[i] != 0){
      j++;
    }
    
    previousVals[i + 1] = previousVals[i];
    
    sum += previousVals[i + 1];
  }
  
  previousVals[0] = dist;
  
  if (j == 0){
    j++;
  }
  
  int average = (int)((sum + dist)/ j);
  return Math.min(250, Math.abs(average));
}

//sets status provided in the parameter
private void setStatus(String stat){
  status = stat;
}


public String getStatus(){
  return status;
}

private void leftTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorLow);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
}

private void fastLeftTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh + 200); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorLow);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
}

private void rightTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorLow); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
  
}

private void fastRightTurn(){
  WallFollowingLab.rightMotor.setSpeed(motorLow); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh + 200);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
  
}

private void goBackwards(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh);
  WallFollowingLab.rightMotor.backward();
  WallFollowingLab.leftMotor.backward();
}

private void setFast(){
  WallFollowingLab.rightMotor.setSpeed(motorHigh); // Start robot moving forward
  WallFollowingLab.leftMotor.setSpeed(motorHigh);
  WallFollowingLab.rightMotor.forward();
  WallFollowingLab.leftMotor.forward();
}

}
