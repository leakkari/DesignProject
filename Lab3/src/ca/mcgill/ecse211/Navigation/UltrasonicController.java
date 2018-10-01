package ca.mcgill.ecse211.Navigation;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
