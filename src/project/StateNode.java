// Thomas Jansen
package project;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

// A class to represent state nodes
public class StateNode {
	
	// Children and dad
	public List<StateNode> children = new ArrayList<StateNode>();
	public StateNode dad;

	// The location of the end effector and state of the robot
	Vector3d location = new Vector3d();
	double theta1;
	double theta2;
	double theta3;
	double length;
	
	// A constructor
	public StateNode(Vector3d loc, double t1, double t2, double t3, double l) {
		location.set(loc);
		theta1 = t1;
		theta2 = t2;
		theta3 = t3;
		length = l;
	}
	
	// Add a child node
	public void add(StateNode n){
		children.add(n);
		n.dad = this;
	}
	
	// Clear a node
	public void clear(){
		children.clear();
		location.set(0,0,0);
		theta1 = theta2 = theta3 = length = 0;
	}
	
	// Traverse the nodes to the one closest to target
	public StateNode traverse(Vector3d target) {
		
		Vector3d dist = new Vector3d();
		dist.set(target);
		dist.sub(location);
		
		Vector3d dist2 = new Vector3d();
		
		StateNode ret = this;
		StateNode temp;
		
		for (StateNode n : children){
			temp = n.traverse(target);
			dist2.set(target);
			dist2.sub(temp.location);
			
			if(dist2.length() < dist.length()) {
				ret = temp;
				dist = dist2;
			}			
		}
		
		return ret;
	}
	
    static final public GLUT glut = new GLUT();
	
    // Display the nodes and their connections
	public void display( GLAutoDrawable drawable ) {		
        GL2 gl = drawable.getGL().getGL2();

        // Draw a sphere for the node
        gl.glPushMatrix();
        gl.glTranslated(location.x, location.y, location.z);
        glut.glutSolidSphere(0.01, 20, 20);
        gl.glPopMatrix();
        
        // For each child connect a cylinder to it
        for (StateNode n : children) {
        	
        	gl.glPushMatrix();
        	
        	// Translate to the correct location
        	gl.glTranslated(location.x, location.y, location.z);
        	
        	// Get the direction from one point to the next
        	Vector3d dist = new Vector3d();
        	dist.set(n.location);
        	dist.sub(location);
        	
        	// Cylinders are drawn along the the z-axis
        	Vector3d axis = new Vector3d();
        	axis.set(0,0,1);
        	
        	// Calculate the angle by which the cylinder needs to rotate
        	double angle = Math.acos(axis.dot(dist) / dist.length());
        	angle = angle * 180 / Math.PI;
        	
        	// Find the axis of rotation
        	axis.cross(axis, dist);
        	
        	// Rotate
        	if (axis.length() != 0.0) { 
        		axis.normalize();
        		gl.glRotated(angle, axis.x, axis.y, axis.z);
        	}
        	
        	// If the axis and direction align, rotate along arbitrary axis
        	else if (angle != 0.0) {
        		gl.glRotated(angle, 1.0, 0.0, 0.0);
        	}

            glut.glutSolidCylinder(0.005, dist.length(), 20, 20);
            gl.glPopMatrix();
        	
        }
        
        // Continue for all the children
        for (StateNode n : children) {
        	n.display(drawable);
        }
	}
}
