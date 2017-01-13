// Thomas Jansen
package project;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class Tube extends DAGNode {
	
	// The position and size of the tube
	private Vector3d pos = new Vector3d();
	private double radius;
	private double height;
	
	// The constructor inputs:
	// name, position(x,y,z), radius, height
	public Tube(String n, double px, double py, double pz, double r, double h) {
		pos.set(px, py, pz);
		radius = r;
		height = h;
		name = n;
		show = false;
	}
		
	@Override
    public void display(GLAutoDrawable drawable) {        
        GL2 gl = drawable.getGL().getGL2();
        
        // Move to the correct position and draw the tube
        gl.glTranslated(pos.x, pos.y, pos.z);
        glut.glutSolidCylinder(radius, height, 20, 20);                
        super.display(drawable);
	}    
}
