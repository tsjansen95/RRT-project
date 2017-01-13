// Thomas Jansen
package project;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class Sphere extends DAGNode {
	
	// The position and size of the sphere
	private Vector3d pos = new Vector3d();
	private double radius;
	
	// The constructor inputs:
	// name, position(x,y,z), radius
	public Sphere(String n, double px, double py, double pz, double r) {
		pos.set(px, py, pz);
		radius = r;
		name = n;
		show = false;
	}
		
	@Override
    public void display(GLAutoDrawable drawable) {        
        GL2 gl = drawable.getGL().getGL2();
        
        // Move to the correct position and draw the sphere
        gl.glTranslated(pos.x, pos.y, pos.z);
        glut.glutSolidSphere(radius, 20, 20);        
        super.display(drawable);
	}    	
}
