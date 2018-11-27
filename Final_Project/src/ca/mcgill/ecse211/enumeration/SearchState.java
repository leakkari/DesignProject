package ca.mcgill.ecse211.enumeration;
/** Enumeration for the searching state
 * This class is used to keep track of the robot state: In_Progress means that the rings hasnt been found yet,
 * the robot is till searching
 * Time_out means the robot is taking a lot of time to find the rings
 * Ring_found state is set when the rings have been collected from the tree
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public enum SearchState {
	IN_PROGRESS,
	TIME_OUT,
	RING_FOUND
}
