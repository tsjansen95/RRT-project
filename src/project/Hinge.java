// Thomas Jansen
package project;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;

public class Hinge extends DAGNode {
	
	public Vector3d pos = new Vector3d();		// The position of the node
	public Vector3d hAxis = new Vector3d();		// The orientation of the rotation axis
	
	// The rotation parameters
	public DoubleParameter hingej;
	
	// The constructor input:
	// name, position(x,y,z), axis of rotation(x,y,z), starting angle of rotation
	public Hinge(String n, double px, double py, double pz, double ax, double ay, double az, double angle) {
		pos.set(px, py, pz);
		hAxis.set(ax, ay, az);
		hingej = new DoubleParameter( "Hinge", angle, -360, 360 ); 
		dofs.add(hingej);
		name = n;
	}
	
	
	@Override
    public void display(GLAutoDrawable drawable) {   
        GL2 gl = drawable.getGL().getGL2();
        
        gl.glPushMatrix();
        
        // translate to the position, then rotate accordingly
        gl.glTranslated(pos.x, pos.y, pos.z);
        gl.glRotated(hingej.getValue(), hAxis.x, hAxis.y, hAxis.z);
        
        super.display(drawable);
        gl.glPopMatrix();
	}    	
}
