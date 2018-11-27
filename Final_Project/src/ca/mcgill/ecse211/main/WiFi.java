package ca.mcgill.ecse211.main;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.enumeration.Team;

/** This class serves to fetch all data from the server JAR
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */

public class WiFi {


	private static final String SERVER_IP = "192.168.2.15";

	private static final int TEAM_NUMBER = 15;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

	// Create Map variable
	private Map data;

	@SuppressWarnings("rawtypes")
	public WiFi() {
		// Store the data
		this.getData();

		// Clear the console
		System.out.flush();
	}

	/**
	 * Stores all the data sent by the server JAR file into a Map object.
	 */
	public void getData() {

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might
		// occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA
			 * presses the "Start" button in the GUI on their laptop with the
			 * data filled in. Once it's waiting, you can kill it by pressing
			 * the upper left hand corner button (back/escape) on the EV3.
			 * getData() will throw exceptions if it can't connect to the server
			 * (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception
			 * if it connects but receives corrupted data or a message from the
			 * server saying something went wrong. For example, if TEAM_NUMBER
			 * is set to 1 above but the server expects teams 17 and 5, this
			 * robot will receive a message saying an invalid team number was
			 * specified and getData() will throw an exception letting you know.
			 */
			data = conn.getData();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
	}

	/**
	 * Gets the color of the team you are in.
	 * 
	 * @return Team color enumeration for the team you are in
	 */
	public Team getTeam() {
		// Return the corresponding team colour for the team number (8)
		if (((Long) data.get("RedTeam")).intValue() == TEAM_NUMBER) {
			return Team.RED;
		} else if (((Long) data.get("GreenTeam")).intValue() == TEAM_NUMBER) {
			return Team.GREEN;
		} else {
			return null;
		}
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return starting corner 
	 */
	public int getStartingCorner(Team team) {
		// Check which team we are
		if (team == Team.RED) {
			return ((Long) data.get("RedCorner")).intValue();
		} else if (team == Team.GREEN) {
			return ((Long) data.get("GreenCorner")).intValue();
		}
		return -1;
	}

	/**BETA DEMO
	 * Gets the starting corner coordinates of your team.
	 * coords(x,y)
	 * @return starting corner coordinates
	 */
	public int[] getStartingCornerCoords(int startingCorner) {
		int[] coords = { 0, 0 };
		switch (startingCorner) {
		case 0:
			coords[0] = 1;
			coords[1] = 1;
			break;
		case 1:
			coords[0] = 7;
			coords[1] = 1;
			break;
		case 2:
			coords[0] = 7;
			coords[1] = 7;
			break;
		case 3:
			coords[0] = 1;
			coords[1] = 7;
			break;
		}
		return coords;
	}

	/**
	 * Gets the coordinates of the home zone depending team color
	 * 
	 * @return 2-D array containing coordinates of the 4 corners of the home zone
	 */
	public int[][] getHomeZone(Team team) {
		int llx,lly,urx,ury;
		switch(team) {
		case GREEN:
			// Get coords of green zone
			llx = ((Long) data.get("Green_LL_x")).intValue();
			lly = ((Long) data.get("Green_LL_y")).intValue();
			urx = ((Long) data.get("Green_UR_x")).intValue();
			ury = ((Long) data.get("Green_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] greenZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

			return greenZone;
		case RED:
			// Get coords of red zone
			llx = ((Long) data.get("Red_LL_x")).intValue();
			lly = ((Long) data.get("Red_LL_y")).intValue();
			urx = ((Long) data.get("Red_UR_x")).intValue();
			ury = ((Long) data.get("Red_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] redZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

			return redZone;

		default : 
			return  null;
		}

	}


	/**
	 * Gets the coordinates of the tunnel.
	 * 
	 * @return 2-D array containing coordinates of the 4 corners of the tunnel
	 */
	public int[][] getTunnelZone(Team team) {
		int llx, lly, urx, ury;
		switch (team) {
		case RED:
			// Get coords of red search zone
			llx = ((Long) data.get("TNR_LL_x")).intValue();
			lly = ((Long) data.get("TNR_LL_y")).intValue();
			urx = ((Long) data.get("TNR_UR_x")).intValue();
			ury = ((Long) data.get("TNR_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] redTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return redTunnelZone;

		case GREEN:
			// Get coords of red search zone
			llx = ((Long) data.get("TNG_LL_x")).intValue();
			lly = ((Long) data.get("TNG_LL_y")).intValue();
			urx = ((Long) data.get("TNG_UR_x")).intValue();
			ury = ((Long) data.get("TNG_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] greenTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return greenTunnelZone;
		default:
			return null;
		}
	}

	/**
	 * Determines the orientation of the tunnel
	 * vertical(along y axis) horizontal(along x-axis)
	 * @return boolean true for vertical, false for horizontal
	 */
	public boolean isTunnelVertical(Team team) {

		int [][] tunnelZone = this.getTunnelZone(team);
		int [][] teamZone = this.getHomeZone(team);
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		switch(this.getStartingCorner(team)) {
		case 0:{
			//compare lly of tunnel to ury of team zone 
			//compare llx & urx of tunnel to urx of team zone
			if(tunnelZone[0][1] <= teamZone[2][1] && tunnelZone[1][1] <= teamZone[2][1]) {
				if(tunnelZone[0][0] >= teamZone[3][0] &&tunnelZone[1][0]<= teamZone[2][0]) {
					return true;
				}
			}
			return false;
		}
		case 1:{
			if(tunnelZone[0][1] <= teamZone[2][1] && tunnelZone[1][1] <= teamZone[2][1]) {
				if(tunnelZone[0][0] >= teamZone[3][0] &&tunnelZone[1][0]<= teamZone[2][0]) {
					return true;
				}
			}
			return false;
		}
		case 2:{
			if(tunnelZone[2][1] >= teamZone[0][1] && tunnelZone[3][1] >= teamZone[0][1]) {
				if(tunnelZone[2][0] <= teamZone[1][0] &&tunnelZone[3][0]>= teamZone[0][0]) {
					return true;
				}
			}
			return false;
		}
		case 3:{
			if(tunnelZone[2][1] >= teamZone[0][1] && tunnelZone[3][1] >= teamZone[0][1]) {
				if(tunnelZone[2][0] <= teamZone[1][0] &&tunnelZone[3][0]>= teamZone[0][0]) {
					return true;
				}
			}
			return false;
		}
		default: return false;
		}
	}


	public int getTunnelSize(Team team) {
		int tunnelSize = 0;
		int llx, lly, urx, ury;
		switch (team) {
		case RED:
			// Get coords of red search zone
			llx = ((Long) data.get("TNR_LL_x")).intValue();
			lly = ((Long) data.get("TNR_LL_y")).intValue();
			urx = ((Long) data.get("TNR_UR_x")).intValue();
			ury = ((Long) data.get("TNR_UR_y")).intValue();

			if(urx-llx >= ury - lly) {
				return (urx-llx);
			}
			else {
				return (ury - lly);
			}


		case GREEN:
			// Get coords of red search zone
			llx = ((Long) data.get("TNG_LL_x")).intValue();
			lly = ((Long) data.get("TNG_LL_y")).intValue();
			urx = ((Long) data.get("TNG_UR_x")).intValue();
			ury = ((Long) data.get("TNG_UR_y")).intValue();

			if(urx-llx >= ury - lly) {
				return (urx-llx);
			}
			else {
				return (ury - lly);
			}

		default:
			return 0;
		}

	}


	/**
	 * Gets the search zone of the specified team
	 * 
	 * @param team the team of the search zone wanted
	 * @return 1-D array containing coordinates of the ring set
	 */
	public int[] getRingSet(Team team) {
		int tr_x,tr_y,tg_x,tg_y;

		switch (team) {
		case RED:
			// Get coords of red search zone
			tr_x = ((Long) data.get("TR_x")).intValue();
			tr_y = ((Long) data.get("TR_y")).intValue();


			// Corner convention:
			// TR_x corresponds to the x coordinate of the red team's tree.
			// TR_y corresponds to the y coordinate of the red team's tree.

			int[] redRingSet = {tr_x, tr_y};

			return redRingSet;

		case GREEN:
			// Get coords of red search zone
			tg_x = ((Long) data.get("TG_x")).intValue();
			tg_y = ((Long) data.get("TG_y")).intValue();

			// Corner convention:
			// TG_x corresponds to the x coordinate of the green team's tree.
			// TG_y corresponds to the y coordinate of the green team's tree.

			int[]greenRingSet = {tg_x,tg_y};

			return greenRingSet;

		default:
			return null;
		}
	}
	/**
	 * Gets the closest corner of the tunnel to ringset
	 * 
	 */
	public int[] getClosestCornerToRS(Team team) {
		int[] coords = { 0, 0, 0 };
		int [] ringSet = getRingSet(team);
		int[][] tunnelZone = getTunnelZone(team);
		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		double min = Double.MAX_VALUE;

		for(int i =0; i < 4; i++) {
			int x =Math.abs(ringSet[0]-tunnelZone[i][0]);
			int y =Math.abs(ringSet[1]-tunnelZone[i][1]);
			double hypo = Math.sqrt((x*x+y*y));
			if(min > hypo ) {
				min = hypo;
				coords[0]=tunnelZone[i][0];
				coords[1]=tunnelZone[i][1];
				coords[2]=i;

			}
		}
		return coords;

	}
	/**
	 * Gets the closest corner of the tunnel to starting corner
	 * 
	 */
	public int[] getClosestCornerToSC(Team team) {
		int[] coords = { 0, 0, 0 };
		int [] sc = getStartingCornerCoords(this.getStartingCorner(team));
		int[][] tunnelZone = getTunnelZone(team);
		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		double min = Double.MAX_VALUE;

		for(int i =0; i < 4; i++) {
			int x =Math.abs(sc[0]-tunnelZone[i][0]);
			int y =Math.abs(sc[1]-tunnelZone[i][1]);
			double hypo = Math.sqrt((x*x+y*y));
			if(min > hypo ) {
				min = hypo;
				coords[0]=tunnelZone[i][0];
				coords[1]=tunnelZone[i][1];
				coords[2]=i;
			}
		}
		return coords;
	}

	/**
	 * Checks if ringSet is in front of the tunnel
	 * 
	 */
	public boolean checkRSPos(Team team) {
		int[] ringSet = getRingSet(team);
		int[][] tunnelZone = getTunnelZone(team);
		int llx = tunnelZone[0][0];
		int lly = tunnelZone[0][1];
		int urx = tunnelZone[2][0];
		int ury = tunnelZone[2][1];


		if(isTunnelVertical(team)) {
			if(ringSet[0]== llx || ringSet[0] == urx) {
				return true;
			}
		}
		else {
			if(ringSet[1]==lly || ringSet[1]==ury) {
				return true;
			}
		}
		return false;
	}
	/**
	 * Checks if tunnel is touching border
	 * 
	 */
	public boolean checkTunnel(Team team) {
		int[][] tunnelZone = getTunnelZone(team);
		int llx = tunnelZone[0][0];
		int lly = tunnelZone[0][1];
		int urx = tunnelZone[2][0];
		int ury = tunnelZone[2][1];


		if(isTunnelVertical(team)) {
			if( llx ==0 ||  urx == 8) {
				return true;
			}
		}
		else {
			if(lly == 0|| ury == 8) {
				return true;
			}
		}
		return false;
	}





}