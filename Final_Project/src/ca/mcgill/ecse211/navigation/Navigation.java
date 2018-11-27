package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.*;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used for navigating the robot, it helps the robot go to a specific tunnel, then travel through the tunnel
 * Then find the ring set and navigate around the ring set.
 * Then go back to the same tunnel, travel through it back to starting point.
 * FOR BETA DEMO 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class Navigation extends Thread {

	//Robot 
	private RobotController robot;

	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_LENGTH;
	private boolean navigate = true;


	//Odometer class
	private Odometer odometer;

	private OdometryCorrection odoCorr;

	//RingSearcher class
	private RingSearcher ringSearcher;

	//Wifi class
	private WiFi wifi;

	//Starting corner
	int startingCorner;
	int[] startingCornerCoords;

	//Tunnel coordinates
	private int[][] tunnelZone;

	//Search zone
	private int[][] searchZone;

	//Home zone
	private int[][] homeZone;

	//Ring set
	private int[] ringSet;

	//Team
	private Team team;

	//Position of the robot in respect to the tunnel
	private boolean upperLeft = false;
	private boolean upperRight = false;
	private boolean lowerLeft = false;
	private boolean lowerRight = false;

	private boolean goingToRingSet = true;
	
	/**
	 * This is a constructor for the RobotController class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param ringSearcher
	 * @param gyroSensor
	 */
	public Navigation(Odometer odometer,RobotController robot, RingSearcher ringSearcher, WiFi wifi) {
		this.odometer = odometer;
		this.robot = robot;
		this.ringSearcher = ringSearcher;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.TILE_SIZE = robot.TILE_SIZE;
		this.SENSOR_LENGTH = robot.SENSOR_LENGTH;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.startingCorner = wifi.getStartingCorner(team);
		this.startingCornerCoords = wifi.getStartingCornerCoords(startingCorner);
		this.homeZone = wifi.getHomeZone(team);
		this.tunnelZone = wifi.getTunnelZone(team);
		this.ringSet = wifi.getRingSet(team);
	}


	/**
	 * A method to drive our vehicle to the tunnel
	 * Travels to the closest point to the tunnel
	 */
	public void travelToTunnel() {
				
		int[] closestCorner = wifi.getClosestCornerToSC(team);

		//tunnel is along y-axis (vertical)
		if(wifi.isTunnelVertical(team)) {
			switch(closestCorner[2]){
			case 0: // LL
				//travel to lower left corner of tunnel
				robot.travelTo(closestCorner[0],startingCornerCoords[1]);
				robot.travelTo(closestCorner[0], closestCorner[1]-1);
				lowerLeft = true;
				robot.checkAngle(0); //assure that robot is pointing 0
				break;
			case 1:	// LR
				//travel to the point under lower-right corner 
				robot.travelTo(closestCorner[0],startingCornerCoords[1]);
				robot.travelTo(closestCorner[0], closestCorner[1]-1);
				lowerRight = true;
				robot.checkAngle(0); // assure that robot is pointing 0
				break;
			case 2: // UR
				//travel to upper right corner
				robot.travelTo(closestCorner[0],startingCornerCoords[1]);
				robot.travelTo(closestCorner[0], closestCorner[1]+1);
				upperRight = true;
				robot.checkAngle(180);
				break;
			case 3: // UL
				//travel to upper left corner
				robot.travelTo(closestCorner[0],startingCornerCoords[1]);
				robot.travelTo(closestCorner[0], closestCorner[1]+1);
				upperLeft = true;
				robot.checkAngle(180);
				break;
			}	
		}
		//tunnel is along x-axis (horizontal)
		else {
			switch(closestCorner[2]){
			case 0: //LL
				//travel to the point under lower-left corner 
				robot.travelTo(startingCornerCoords[0], closestCorner[1]);
				robot.travelTo(closestCorner[0]-1,closestCorner[1]);
				lowerLeft = true;
				robot.checkAngle(90); // assure that robot is pointing 90
				break;
			case 1: //LR
				//travel to the point under lower-right corner 
				robot.travelTo(startingCornerCoords[0], closestCorner[1]);
				robot.travelTo(closestCorner[0]+1,closestCorner[1]);
				lowerRight = true;
				robot.checkAngle(270); // assure that robot is pointing 270
				break;				 
			case 2: //UR
				//travel to the point under upper-right corner 
				robot.travelTo(startingCornerCoords[0], closestCorner[1]);
				robot.travelTo(closestCorner[0]+1,closestCorner[1]);
				upperRight = true;
				robot.checkAngle(270); // assure that robot is pointing 270
				break;				 
			case 3: //UL
				//travel to the point under upper-left corner 
				robot.travelTo(startingCornerCoords[0], closestCorner[1]);
				robot.travelTo(closestCorner[0]-1,closestCorner[1]);
				upperLeft = true;
				robot.checkAngle(90); // assure that robot is pointing 90
				break;
			}
		}
	}

	/**
	 * A method to travel through the tunnel
	 * 
	 */
	public void travelThroughTunnel(){

		turnToTunnel();

		//Correct at initial line
		odoCorr.correct(odometer.getXYT()[2]);

		robot.travelDist(TILE_SIZE);

		//Correct at line at the tunnel entrance
		odoCorr.correct(odometer.getXYT()[2]);

		//Move in the tunnel
		robot.resetMotors();
		
		robot.setSpeeds(250,250);
		int tunnelSize = wifi.getTunnelSize(team);
		robot.travelDist(tunnelSize*TILE_SIZE);
		
		//Correct at line at the tunnel exit
		odoCorr.correct(odometer.getXYT()[2]);
		
		robot.travelDist(TILE_SIZE);

		//Correct at next line
		odoCorr.correct(odometer.getXYT()[2]);
		
		robot.travelDist(SENSOR_LENGTH);

		turnOutTunnel(goingToRingSet);
		
		if(goingToRingSet) {
			goingToRingSet = false;
		}
		
	}
	/**
	 * A method to turn correctly to the tunnel entrance/exit
	 * 
	 */
	public void turnToTunnel() {
		//Tunnel is vertical
		if(wifi.isTunnelVertical(team)) {
			if(upperLeft) {
				robot.turnBy(90,false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,true);
				upperLeft = false; //reinitialize
			}
			//Red tunnel along y-axis
			else if (upperRight) {
				robot.turnBy(90,true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,false);
				upperRight = false; //reinitialize
			}
			//Green tunnel along y-axis
			else if (lowerLeft) {
				robot.turnBy(90,true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,false);
				lowerLeft = false; //reinitialize
			}
			//Green tunnel along x-axis
			else if(lowerRight) {
				robot.turnBy(90,false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,true);
				lowerRight = false; //reinitialize
			}
		}
		else {
			if(upperLeft) {
				robot.turnBy(90,true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,false);
				upperLeft = false; //reinitialize
			}
			//Red tunnel along y-axis
			else if (upperRight) {
				robot.turnBy(90,false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,true);
				upperRight = false; //reinitialize
			}
			//Green tunnel along y-axis
			else if (lowerLeft) {
				robot.turnBy(90,false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,true);
				lowerLeft = false; //reinitialize
			}
			//Green tunnel along x-axis
			else if(lowerRight) {
				robot.turnBy(90,true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(TILE_SIZE/2+SENSOR_LENGTH);
				robot.turnBy(90,false);
				lowerRight = false; //reinitialize
			}
		}
		

	}
	/**
	 * A method to turn correctly after traveling through tunnel
	 * When it goes to the island
	 * 
	 */
	public void turnOutTunnel(boolean goingToRingSet) {
		int[] closestCorner;
		int corner;
		if(goingToRingSet) {
			closestCorner = wifi.getClosestCornerToRS(team);
			corner = closestCorner[2];
			//Check if ringset is in front of the tunnel (true: rs is in front)
			if(wifi.checkRSPos(team)) {
				if(wifi.isTunnelVertical(team)) {
					switch(corner) {
					case 0: corner = 1; closestCorner[0]++;break;
					case 1:	corner = 0; closestCorner[0]--;break;
					case 2: corner = 3; closestCorner[0]--;break;
					case 3: corner = 2; closestCorner[0]++;break;
					}
				}
				else {
					switch(corner) {
					case 0: corner = 3; closestCorner[1]++; break;
					case 1:	corner = 2; closestCorner[1]++;break;
					case 2: corner = 1; closestCorner[1]--;break;
					case 3: corner = 0; closestCorner[1]--;break;
					}	
				}
			}
		}
		else {
			closestCorner = wifi.getClosestCornerToSC(team);
			corner = closestCorner[2];
		}

		if(wifi.isTunnelVertical(team)) {
			switch(corner) {
			case 0: //LL
				robot.turnBy(90, true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, false);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT(closestCorner[0]*TILE_SIZE,(closestCorner[1]-1)*TILE_SIZE, 180);
				break;
			case 1: //LR
				robot.turnBy(90, false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, true);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT(closestCorner[0]*TILE_SIZE,(closestCorner[1]-1)*TILE_SIZE, 180);
				break;
			case 2: //UR
				robot.turnBy(90, true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, false);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT(closestCorner[0]*TILE_SIZE,(closestCorner[1]+1)*TILE_SIZE, 0);
				break;
			case 3: //UL
				robot.turnBy(90, false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, true);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT(closestCorner[0]*TILE_SIZE,(closestCorner[1]+1)*TILE_SIZE, 0);
				break;
			}
		}
		else {
			switch(corner) {
			case 0: //LL
				robot.turnBy(90, false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, true);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT((closestCorner[0]-1)*TILE_SIZE,closestCorner[1]*TILE_SIZE, 270);
				break;
			case 1: //LR
				robot.turnBy(90, true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, false);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT((closestCorner[0]+1)*TILE_SIZE,closestCorner[1]*TILE_SIZE, 90);
				break;
			case 2: //UR
				robot.turnBy(90, false);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, true);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT((closestCorner[0]+1)*TILE_SIZE,closestCorner[1]*TILE_SIZE, 90);
				break;
			case 3: //UL
				robot.turnBy(90, true);
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				robot.turnBy(90, false);
				//odoCorr.correct(odometer.getXYT()[2]);
				//robot.travelDist(SENSOR_LENGTH);
				odometer.setXYT((closestCorner[0]-1)*TILE_SIZE,(closestCorner[1])*TILE_SIZE, 270);
				break;
			}	
		}
	}
	/**
	 * A method to travel to the ringSet
	 * 
	 */
	public void travelToRingSet() {
		int currx=0, curry=0;
		int [] closestCorner = wifi.getClosestCornerToRS(team);
		//closestCorner[0] : x - coords
		//closestCorner[1] : y - coords
		//closestCorner[2] : corner of the tunnel (LL(0),LR(1),UR(2),UL(3)

		//Tunnel along y-axis (vertical)
		if(wifi.isTunnelVertical(team)) {
			switch(closestCorner[2]) { 
			case 0: //LL
				currx = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(currx,ringSet[1]);
				robot.travelTo(ringSet[0]+1,ringSet[1]);
				robot.checkAngle(270);
				break;
			case 1: //LR
				currx = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(currx,ringSet[1]);
				robot.travelTo(ringSet[0]-1,ringSet[1]);
				robot.checkAngle(90);
				break;
			case 2: //UR
				currx = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(currx,ringSet[1]);
				robot.travelTo(ringSet[0]-1,ringSet[1]);
				robot.checkAngle(90);
				break;
			case 3: //UL
				currx = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(currx,ringSet[1]);
				robot.travelTo(ringSet[0]+1,ringSet[1]);
				robot.checkAngle(270);
				break;
			}
		}
		//Tunnel along x-axis (horizontal)
		else {
			switch(closestCorner[2]) { 
			case 0: //LL
				curry = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(ringSet[0],curry);
				robot.travelTo(ringSet[0],ringSet[1]+1);
				robot.checkAngle(180);
				break;
			case 1: //LR
				curry = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(ringSet[0],curry);
				robot.travelTo(ringSet[0],ringSet[1]+1);
				robot.checkAngle(180);
				break;
			case 2: //UR
				curry = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(ringSet[0],curry);
				robot.travelTo(ringSet[0],ringSet[1]-1);
				robot.checkAngle(0);
				break;
			case 3: //UL
				curry = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(ringSet[0],curry);
				robot.travelTo(ringSet[0],ringSet[1]-1);
				robot.checkAngle(0);
				break;
			}
		}
	}
	/**
	 * A method to travel to tunnel exit
	 * 
	 */
	public void travelToTunnelExit() {
		int[] closestCorner = wifi.getClosestCornerToRS(team);
		int corner = closestCorner[2];
		//closestCorner[0] : x - coords
		//closestCorner[1] : y - coords
		//closestCorner[2] : corner of the tunnel (LL(0),LR(1),UR(2),UL(3)
		if(wifi.checkRSPos(team)) {
			if(wifi.isTunnelVertical(team)) {
				switch(corner) {
				case 0: corner = 1; closestCorner[0]++;break;
				case 1:	corner = 0; closestCorner[0]--;break;
				case 2: corner = 3; closestCorner[0]--;break;
				case 3: corner = 2; closestCorner[0]++;break;
				}
			}
			else {
				switch(corner) {
				case 0: corner = 3; closestCorner[1]++;break;
				case 1:	corner = 2; closestCorner[1]++;break;
				case 2: corner = 1; closestCorner[1]--;break;
				case 3: corner = 0; closestCorner[1]--;break;
				}	
			}
		}
		if(wifi.isTunnelVertical(team)) {
			switch(corner) { 
			case 0:{ //LL
				//x then y
				int y = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(closestCorner[0], y);
				robot.travelTo(closestCorner[0], closestCorner[1]-1);
				robot.checkAngle(0);
				lowerLeft = true;
				break;
			}
			case 1:{ //LR
				//x then y
				int y = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(closestCorner[0], y);
				robot.travelTo(closestCorner[0], closestCorner[1]-1);
				robot.checkAngle(0);
				lowerRight = true;
				break;
			}
			case 2:{ //UR
				//x then y
				int y = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(closestCorner[0], y);
				robot.travelTo(closestCorner[0], closestCorner[1]+1);
				robot.checkAngle(180);
				upperRight = true;
				break;
			}
			case 3:{ //UL
				//x then y
				int y = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
				robot.travelTo(closestCorner[0], y);
				robot.travelTo(closestCorner[0], closestCorner[1]+1);
				robot.checkAngle(180);
				upperLeft = true;
				break;
			}
			}
		}
		else {
			switch(corner) {
			case 0:{ //LL
				//y then x
				int x = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(x, closestCorner[1]);
				robot.travelTo(closestCorner[0]-1, closestCorner[1]);
				robot.checkAngle(90);
				lowerLeft = true;
				break;
			}
			case 1:{ //LR
				int x = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(x, closestCorner[1]);
				robot.travelTo(closestCorner[0]+1, closestCorner[1]);
				robot.checkAngle(270);
				lowerRight = true;
				break;
			}
			case 2:{ //UR
				int x = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(x, closestCorner[1]);
				robot.travelTo(closestCorner[0]+1, closestCorner[1]);
				robot.checkAngle(270);
				upperRight = true;
				break;
			}
			case 3:{ //UL
				//y then x
				int x = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
				robot.travelTo(x, closestCorner[1]);
				robot.travelTo(closestCorner[0]-1, closestCorner[1]);
				robot.checkAngle(90);
				upperLeft = true;
				break;
			}
			}
		}
	}
	/**
	 * A method to travel to the starting point
	 * 
	 */
	public void travelToStartingPoint() {
		int currx=0, curry=0;
		int [] closestCorner = wifi.getClosestCornerToSC(team);
		//closestCorner[0] : x - coords
		//closestCorner[1] : y - coords
		//closestCorner[2] : corner of the tunnel (LL(0),LR(1),UR(2),UL(3)

		//Tunnel along y-axis (vertical)
		if(wifi.isTunnelVertical(team)) {
			currx = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
			robot.travelTo(currx,startingCornerCoords[1]);
			robot.travelTo(startingCornerCoords[0],startingCornerCoords[1]);
		}
		//Tunnel along x-axis (horizontal)
		else {
			curry = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);
			robot.travelTo(startingCornerCoords[0],curry);
			robot.travelTo(startingCornerCoords[0],startingCornerCoords[1]);
		}
	}

	/**
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}

}

