// Thomas Jansen
package project;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

// This class creates collidable spheres
public class Collidable {
	
	// The location and radius of the sphere
	Vector3d location = new Vector3d();
	double radius = 0;

	// The constructor
	public Collidable(Vector3d loc, double r) {
		location.set(loc);
		radius = r;
	}
	
	// Sphere-sphere collisions with the given sphere
	public boolean SphereSphere(Vector3d l, double r, double epsilon) {
		
		Vector3d dist = new Vector3d();
		dist.set(l);
		dist.sub(location);
		
		double dubR = r + radius;
		
		if (dist.length() < dubR + epsilon) { return true; }		
		return false;		
	}
	
	// Sphere-sphere collisions with the given sphere
	public static boolean SphereSphere(Vector3d l, Vector3d l2, double r, double r2, double epsilon) {
		
		Vector3d dist = new Vector3d();
		dist.set(l);
		dist.sub(l2);
		
		double dubR = r + r2;
		
		if (dist.length() < dubR + epsilon) { return true; }		
		return false;		
	}
	
	// Sphere-Cylinder collisions
	public boolean SphereCylinder( Vector3d p1, Vector3d p2, double r, double epsilon) {
		
		// Find the axis of the cylinder
		Vector3d cylDir = new Vector3d();
		cylDir.set(p2);
		cylDir.sub(p1);
		cylDir.normalize();
		
		// Find the axis from p1 to this sphere
		Vector3d toLoc = new Vector3d();
		toLoc.set(location);
		toLoc.sub(p1);
		double d1 = toLoc.length();
		toLoc.normalize();
		
		Vector3d answer = new Vector3d();
		
		// Check distance if they are perpendicular
		if (cylDir.dot(toLoc) == 0.0) {			
			answer.set(p1);
			answer.sub(location);
			if (answer.length() < r + radius + epsilon ) { return true; }
			return false;
		}
		
		double alpha = Math.acos(cylDir.dot(toLoc));
		
		// Check distance from end if perp-distance is before cylinder line segment
		if (alpha < 0.0) {
			answer.set(p1);
			answer.sub(location);
			if (answer.length() < r + radius + epsilon ) { return true; }
			return false;
		}
		
		// Find the details of the position triangle
		double dist = d1 * Math.sin(alpha);
		double dAxis = d1 * Math.cos(alpha);
		
		cylDir.set(p2);
		cylDir.sub(p1);
		
		// Check distance from end point if perp-distance is after cylinder line segment
		if (cylDir.length() < dAxis) {
			answer.set(p2);
			answer.sub(location);
			if (answer.length() < r + radius + epsilon ) { return true; }
			return false;
		}
		
		// Otherwise check perp-distance to cylinder axis
		if (dist < r + radius + epsilon ) { return true; }		
		return false;
	}
	
	// Sphere-plane collision
	public static boolean SpherePlane(Vector3d l, double r, double epsilon) {
		
		Vector3d dist = new Vector3d();
		dist.set(l.x, RRT.plane, l.z);
		dist.sub(l);
		
		if(dist.length() < r + epsilon) { return true; }		
		return false;
	}
	
	// Cylinder-plane collisiont
	public static boolean CylinderPlane(Vector3d p1, Vector3d p2, double r, double epsilon) {
		
		// Impossible for either y location to be below axis
		if (p1.y < RRT.plane) { return true; }
		if (p2.y < RRT.plane) { return true; }
		
		// Otherwise check proximity to lowest point on cylinder
		Vector3d low = p1;
		if (p2.y < p1.y) { low = p2; }
		
		Vector3d dist = new Vector3d();
		dist.set(low.x, RRT.plane, low.z);
		dist.sub(low);
		
		if(dist.length() < r + epsilon) { return true; }		
		return false;
	}
	
    static final public GLUT glut = new GLUT();
    
    // Display collidables
	public void display( GLAutoDrawable drawable ) {
			
        GL2 gl = drawable.getGL().getGL2();

        // Draw a sphere at given location
		gl.glPushMatrix();
		gl.glTranslated(location.x, location.y, location.z);
        glut.glutSolidSphere(radius, 20, 20);
        gl.glPopMatrix();  
	}
}
