// Thomas Jansen
package project;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;

public class Prismatic extends DAGNode {
	
	public Vector3d pos = new Vector3d();		// The position of the node
	public Vector3d pAxis = new Vector3d();		// The orientation of the axis
	
	// The translation parameter
	public DoubleParameter prismj;
	
	// The constructor input:
	// name, position(x,y,z), axis of translation(x,y,z), starting position along axis
	public Prismatic(String n, double px, double py, double pz, double ax, double ay, double az, double start) {
		pos.set(px, py, pz);
		pAxis.set(ax, ay, az);
		prismj = new DoubleParameter( "Translate", start, 0.0, RRT.hE );
		dofs.add(prismj);
		name = n;
	}	
	
	@Override
    public void display(GLAutoDrawable drawable) {   
        GL2 gl = drawable.getGL().getGL2();
        
        gl.glPushMatrix();
        
        // Translate to the position, then translate accordingly
        gl.glTranslated(pos.x, pos.y, pos.z);
        gl.glTranslated(prismj.getValue() * pAxis.x, prismj.getValue() * pAxis.y, prismj.getValue() * pAxis.z);
        
        super.display(drawable);
        gl.glPopMatrix();
	}    	
}