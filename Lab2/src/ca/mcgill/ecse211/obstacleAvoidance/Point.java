package ca.mcgill.ecse211.obstacleAvoidance;

/**
 * This class implements x and y coordinates 
 * @author leaakkari
 *
 */
public class Point {
	double x; 
	double y;
	
	
	public Point() {
		
	}
	public Point (double x, double y){
		this.x = x;
		this.y = y;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}

}

