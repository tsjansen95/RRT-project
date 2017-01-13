// Thomas Jansen
package project;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

public class RRT {
	
	// Create the key framed scene
    public KeyFramedScene kfs = new KeyFramedScene();
    
    // Create nodes for the starting and ending locations as well as the path
    public StateNode base;
    public StateNode goal;
    public List<StateNode> path = new LinkedList<StateNode>();
    public List<Collidable> objects = new LinkedList<Collidable>();
    
    // An array of current degrees of freedom
    private double dofs[];
    private double pdofs[];
    private Random rand = new Random();
     
    // Variables for the target location
    DoubleParameter x_target = new DoubleParameter( "Target X", rand.nextDouble() * 2.0 - 1.0, -rad, rad );
    DoubleParameter y_target = new DoubleParameter( "Target Y", rand.nextDouble(), plane, rad );
    DoubleParameter z_target = new DoubleParameter( "Target Z", rand.nextDouble() * 2.0 - 1.0, -rad, rad );
    
    // Variables for the variation of each degree of freedom in the random generator
    DoubleParameter t1_target = new DoubleParameter("Theta 1 Max Change", 1.0, 0.1, 4.0);
    DoubleParameter t2_target = new DoubleParameter("Theta 2 Max Change", 1.0, 0.1, 4.0);
    DoubleParameter t3_target = new DoubleParameter("Theta 3 Max Change", 1.0, 0.1, 4.0);
    DoubleParameter l_target = new DoubleParameter("Length Max Change", hE / 20.0, 0.01, hE / 2.0);
    
    // How close the end effector needs to be to the target for a match
    DoubleParameter fDist = new DoubleParameter("Min Distance to Goal", 0.1, 0.01, 0.5);
    
    // Collision parameters
    DoubleParameter sphereSize = new DoubleParameter("Next Sphere Size", 0.1, 0.01, 1.0);
    DoubleParameter epsilon = new DoubleParameter("Espilon for Collisions", 0.02, 0.0, 0.2);
    
    // How close the axis should align for method 1 to kick in, in method 2
    DoubleParameter threshold = new DoubleParameter("Threshold for Prismatic", 2.0, 0.1, 20.0);    
    
    // Maximum number of loops in the random tree generation
    IntParameter maxLoops = new IntParameter("Max Loops", 40000, 1000, 100000);
    
    // A boolean to show the tree
    BooleanParameter show_tree = new BooleanParameter("Show Tree (this may slow the program)", false );
    
    // Choose appropriate ratios for my robot arm
 	public static double rS = 0.1; 			// the radius of the main sphere
 	public static double rA = 0.05;			// the radius of the arm cylinders
 	public static double hA = 0.5;			// the height of the arm cylinders
 	public static double rE = 0.01;			// the radius of the end effector
 	public static double hE = 0.5;			// the height of the end effector
 	public static double plane = -0.1; 		// y-location of the plane
 	public static double rad = 2*hA + hE;	// maximum radius of the arm
    	
	public RRT() {
		
	}	
	
	// Set dofs to the current values
	public void setDofs() {
		dofs = new double[kfs.dofList.size()];
		for (int i = 0; i < kfs.dofList.size(); i++) {
        	dofs[i] = kfs.dofList.get(i).getFloatValue();
        }
	}
	
	public void setPDofs() {
		pdofs = new double[kfs.dofList.size()];
		for (int i = 0; i < kfs.dofList.size(); i++) {
        	pdofs[i] = kfs.dofList.get(i).getFloatValue();
        }
	}
	
	// Find the point at the end of the first arm
	public Vector3d findP1(double theta1, double theta2) {
		
		Vector3d location = new Vector3d();
		
		// Correct the values to within (-180, 180)
		double nt1 = theta1;
		while (nt1 > 180) { nt1 = nt1 - 360; }
		while (nt1 < -180) { nt1 = nt1 + 360; }
		
		double nt2 = theta2;
		while ( nt2 > 180 ) { nt2 = nt2 - 360; }
		while (nt2 < -180) { nt2 = nt2 + 360; }
		
		double ntp = theta2 + 90;
		while ( ntp > 180 ) { ntp = ntp - 360; }
		while (ntp < -180) { ntp = ntp + 360; }
		
		// Solve using spherical coordinates
		location.y = hA * Math.cos(ntp * Math.PI / 180);		
		location.z = Math.sqrt((hA*hA - location.y*location.y) / 
				(1 + Math.pow(Math.tan(nt1 * Math.PI / 180), 2)));
		
		if ((nt1 > 90 || nt1 < -90) && (nt2 < 90 && nt2 > -90)) { location.z = -location.z; }
		if ((nt1 <= 90 && nt1 >= -90) && (nt2 > 90 || nt2 < -90)) { location.z = -location.z; }
		
		location.x = location.z * Math.tan(nt1 * Math.PI / 180);		
		return location;
	}
	
	// Find the point at the end effector (similar to findP1)
	public Vector3d findLocation(double theta1, double theta2, double theta3, double len) {
		
		Vector3d location = findP1(theta1, theta2);
		Vector3d t_loc = new Vector3d();
			
		double ct = theta2 + theta3;
		while ( ct > 180 ) { ct = ct - 360; }
		while (ct < -180) { ct = ct + 360; }
		
		double ctheta = theta2 + theta3 + 90;
		while (ctheta > 180) { ctheta = ctheta - 360; }
		while (ctheta < -180) { ctheta = ctheta + 360; }
		
		double nt1 = theta1;
		while (nt1 > 180) { nt1 = nt1 - 360; }
		while (nt1 < -180) { nt1 = nt1 + 360; }
		
		t_loc.y = (hA + len) * Math.cos(ctheta * Math.PI / 180);		
		t_loc.z = Math.sqrt(((hA+len)*(hA+len) - t_loc.y*t_loc.y) / 
				(1 + Math.pow(Math.tan(nt1 * Math.PI / 180), 2)));	
		
		if (( nt1 > 90 || nt1 < -90 ) && (ct < 90 && ct > -90)) { t_loc.z = -t_loc.z; }
		else if ((nt1 <= 90 && nt1 >= -90) && (ct > 90 || ct < -90)) { t_loc.z = -t_loc.z; }
		
		t_loc.x = t_loc.z * Math.tan(nt1 * Math.PI / 180);
		
		location.x = location.x + t_loc.x;
		location.y = location.y + t_loc.y;
		location.z = location.z + t_loc.z;				
		return location;
	}
	
	// Create the tree and set the key frames
	public void createTree(int type) {
		
		// Set the base to current location
		setDofs();
		base = new StateNode(findLocation(dofs[0], dofs[1], dofs[2], dofs[3]), dofs[0], dofs[1], dofs[2], dofs[3]);
		
		// Generate the tree
		goal = Generate(type);
		
		// Find all state nodes along the path
		kfs.deleteAllKeyFrames();
		path.clear();
		if (goal != null) { createPath(goal); }
		
		// Set the key frames to nodes along the path (spaced appropriately)
		StateNode n;
		if (path.size() < kfs.num_frames) {
			int j = 0;
			if (!path.isEmpty()) { j = kfs.num_frames / path.size(); }
			int k = 0;
			for(int i = 0; i < path.size(); i++){
				
				n = path.get(i);
				kfs.dofList.get(0).setValue(n.theta1);
				kfs.dofList.get(1).setValue(n.theta2);
				kfs.dofList.get(2).setValue(n.theta3);
				kfs.dofList.get(3).setValue(n.length);
				
				kfs.setKeyFrame(k);
				k += j;
			}
		}
		else {
			int j = path.size() / kfs.num_frames;
			int k = 0;
			for(int i = 0; i < path.size(); i++){
				
				n = path.get(k);
				kfs.dofList.get(0).setValue(n.theta1);
				kfs.dofList.get(1).setValue(n.theta2);
				kfs.dofList.get(2).setValue(n.theta3);
				kfs.dofList.get(3).setValue(n.length);
				
				kfs.setKeyFrame(i);
				k += j;
			}
		}
	}
	
	// Generate the tree
	public StateNode Generate(int type) {
		
		// Set the target to the current target
		Vector3d target = new Vector3d();
		target.set(x_target.getFloatValue(), y_target.getFloatValue(), z_target.getFloatValue());
		
		// Find the current distance to the target location
		Vector3d dist = new Vector3d();
		dist.set(target);
		setDofs();
		dist.sub(findLocation(dofs[0], dofs[1], dofs[2], dofs[3]));
		
		// If it's already there, return the base
		if (dist.length() <= fDist.getFloatValue()) { return base; }
		
		// nPos picks random locations in space
		Vector3d nPos = new Vector3d();
		int num = 0;
		
		// Make nodes up to the max allowable number of loops
		for(int i = 0; i < maxLoops.getValue(); i++) {
			
			// Everyone once in a while set random location to specific locations
			if (i%50 == 1) { nPos.set(0, rad + 1.0, 0); }
			else if (i%50 == 11) { nPos.set(rad + 1.0, 0, 0); }
			else if (i%50 == 21) { nPos.set(-rad - 1.0, 0, 0); }
			else if (i%50 == 31) { nPos.set(0, 0, rad + 1.0); }
			else if (i%50 == 41) { nPos.set(0, 0, -rad - 1.0); }
			else if (i%10 == 0) { nPos.set(target); }
			
			// Otherwise choose random point
			else {
				nPos.x = (rand.nextDouble() - 0.5) * (x_target.getMaximum() + 1.0);
				nPos.y = (rand.nextDouble() - 0.5) * (y_target.getMaximum() + 1.0);
				nPos.z = (rand.nextDouble() - 0.5) * (z_target.getMaximum() + 1.0);
			}
			
			// Find the current closest node to nPos
			StateNode curr = base.traverse(nPos);
			dist.set(nPos);
			dist.sub(curr.location);
			
			// Set state variables to curr variables
			double t1 = curr.theta1;
			double t2 = curr.theta2;
			double t3 = curr.theta3;
			double l = curr.length;
			Vector3d temp = new Vector3d();
			
			// Try 10 different random changes in state
			for(int j = 0; j < 20; j++){
				
				double deltaT1 = 0, deltaT2 = 0, deltaT3 = 0, deltaL = 0;
				
				deltaT1 = (rand.nextDouble() - 0.5) * 2.0 * t1_target.getFloatValue();
				deltaT2 = (rand.nextDouble() - 0.5) * 2.0 * t2_target.getFloatValue();
				deltaT3 = (rand.nextDouble() - 0.5) * 2.0 * t3_target.getFloatValue();
				
				// for type 1 just choose a random difference in the prismatic joint
				if (type == 1) {
					
					// Compute random change according to variances
					deltaL = (rand.nextDouble() - 0.5) * 2.0 * l_target.getFloatValue();
				}
				
				// otherwise check how the prismatic should change
				else if (type == 2) {
					
					// Find the angle between the current closest node's orientation and the desired orientation
					Vector3d targ = new Vector3d();
					targ.set(nPos);
					targ.sub(findP1(t1, t2));
					targ.normalize();
					
					Vector3d aim = new Vector3d();
					aim.set(findLocation(t1, t2, t3, l));
					aim.sub(findP1(t1, t2));
					aim.normalize();
					
					double angle = Math.acos(targ.dot(aim));
					
					// if the angle is within the threshold and doesn't collide, extend the prismatic randomly
					if (angle <= threshold.getFloatValue() * Math.PI / 180.0) {
						
						boolean col = false;
						for (Collidable c : objects) {
							if (c.SphereCylinder(findLocation(t1, t2, t3, l), target, rE, epsilon.getFloatValue())) {
								col = true;
							}
						}
						
						if (!col) { deltaL = (rand.nextDouble() - 0.5) * 2.0 * l_target.getFloatValue(); }
						else { deltaL = (rand.nextDouble() - 1.0) * l_target.getFloatValue(); }
					}
					
					// otherwise retract the prismatic
					else { deltaL = (rand.nextDouble() - 1.0) * l_target.getFloatValue(); } 
				}
				
				// Make sure changes are within boundaries
				if(t1 + deltaT1 > -360 && t1 + deltaT1 < 360) { t1 += deltaT1; }
				if(t2 + deltaT2 > -360 && t2 + deltaT2 < 360) { t2 += deltaT2; }
				if(t3 + deltaT3 > -360 && t3 + deltaT3 < 360) { t3 += deltaT3; }
				if(l + deltaL > 0.0 && l + deltaL < hE) { l += deltaL; }
				
				// Find the distance from new states to nPos
				temp.set(nPos);
				temp.sub(findLocation(t1, t2, t3, l));
				
				// If position is closer, keep it, otherwise continue
				if (temp.length() < dist.length()) { 
					
					// Check for collisions on the new node
					if (!collideAll(t1, t2, t3, l)) {
						
						// Create the node and add it
						StateNode n = new StateNode(findLocation(t1, t2, t3, l), t1, t2, t3, l);
						curr.add( n );
						num++;
						
						// If new node is close enough to target, tree is finished, return node
						temp.set(target);
						temp.sub(n.location);
						if (temp.length() < fDist.getFloatValue()) {
							System.out.print("type = " + type + " nodes = " + num + " loops = " + i);
							return n; 
						}
						break;
					}
				}
			}
		}		
		
		System.out.print("type = " + type + " nodes = " + num + " loops = " + maxLoops.getValue() + "\n");
		return null;
	}
	
	// This method will check for collisions against all the robot parts
	public boolean collideAll(double t1, double t2, double t3, double l) {
		
		// Check for collisions against the plane
		Vector3d empty = new Vector3d();
		if (Collidable.CylinderPlane(empty, findP1(t1, t2), 
				rA, epsilon.getFloatValue())) { return true; }
		if (Collidable.SpherePlane(findP1(t1, t2), rA, epsilon.getFloatValue())) { return true; }
		if (Collidable.CylinderPlane(findP1(t1, t2), 
				findLocation(t1, t2, t3, 0.0), rA, epsilon.getFloatValue())) { return true; }
		if (Collidable.SphereSphere(findLocation(t1, t2, t3, 0.0), empty, rA, rA, epsilon.getFloatValue())) {
			return true;
		}
		
		if (l > 0.0) {
			if (Collidable.CylinderPlane(findLocation(t1, t2, t3, 0.0), 
					findLocation(t1, t2, t3, l), rE, epsilon.getFloatValue())) {
				return true;
			}
		}
		
		// Check for collisions against each collidable
		for (Collidable c : objects){
			if (c.SphereCylinder(empty, findP1(t1, t2), rA, epsilon.getFloatValue())) { return true; }
			if (c.SphereSphere(findP1(t1, t2), rA, epsilon.getFloatValue())) { return true; }
			if (c.SphereCylinder(findP1(t1, t2), findLocation(t1, t2, t3, 0.0), 
					rA, epsilon.getFloatValue())) { return true; }
			
			if (l > 0.0) {
				if (c.SphereCylinder(findLocation(t1, t2, t3, 0.0), 
						findLocation(t1, t2, t3, l), rE, epsilon.getFloatValue())) {
					return true;
				}
			}
		}
		return false;
	}
	
	// Create a path from the last node up to the base
	public void createPath(StateNode n) {
		StateNode curr = n;
		while (curr.dad != null) {
			path.add(0, curr);
			curr = curr.dad;
		}
		path.add(0, base);
		System.out.print(" path = " + path.size() + "\n");
	}
	 
	// Create the robot with the given initial states
	static public DAGNode create(double i1, double i2, double i3, double i4) {
		
		// Create the main sphere
		Hinge root = new Hinge("Main Sphere", 0.0, plane, 0.0, 0.0, 1.0, 0.0, i1);
		Sphere body = new Sphere("main sphere", 0.0, 0.0, 0.0, rS);
		
		Sphere fJoint = new Sphere("first joint", 0.0, 0.0, 0.0, rA);
		
		// Create the arms
		Hinge fArmHinge = new Hinge("First Cylinder", 0.0, rS, 0.0, 1.0, 0.0, 0.0, i2);
		Tube fArm = new Tube("first arm", 0.0, 0.0, 0.0, rA, hA);
		
		Sphere sJoint = new Sphere("second joint", 0.0, 0.0, 0.0, rA);
		
		Hinge sArmHinge = new Hinge("Second Cylinder", 0.0, 0.0, hA, 1.0, 0.0, 0.0, i3);
		Tube sArm = new Tube("second arm", 0.0, 0.0, 0.0, rA, hA);
				
		// Create the end effector
		Prismatic ePrism = new Prismatic("End Effector", 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, i4);
		Tube eArm = new Tube("end effector", 0.0, 0.0, 0.0, rE, hE);
		
		// Add all the nodes to their respective parents
		root.add(body);		
		body.add(fArmHinge);
		fArmHinge.add(fJoint);
		fArmHinge.add(fArm);
		fArm.add(sArmHinge);
		sArmHinge.add(sArm);
		sArmHinge.add(sJoint);
		sArm.add(ePrism);
		ePrism.add(eArm);
				    	
		// Return the root
		return root;
	}
	
	// Get the controls for manipulating the robot and tree
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		
		vfp.add(fDist.getSliderControls(false));
		vfp.add(maxLoops.getSliderControls());	
		
		// Add a button to give a random target location
		JButton rtl = new JButton("Random Target Location");
    	rtl.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				
				boolean inside = false;
				double distance;
				Vector3d dist = new Vector3d();
				Vector3d t = new Vector3d();
				
				do {
					do {
						x_target.setValue(rand.nextDouble() * 2.0 * rad - rad);
						y_target.setValue(rand.nextDouble() * (rad - plane) - plane);
						z_target.setValue(rand.nextDouble() * 2.0 * rad - rad);
						distance = Math.sqrt(Math.pow(x_target.getFloatValue(),2) + 
								Math.pow(y_target.getFloatValue(),2) + Math.pow(z_target.getFloatValue(),2));
					} while (distance > rad);
					
					// Make sure the new target isn't inside an obstacle
					t.set(x_target.getFloatValue(), y_target.getFloatValue(), z_target.getFloatValue());
					for (Collidable c : objects) {
						dist.set(c.location);
						dist.sub(t);
						if (dist.length() < c.radius + rA) { 
							inside = true; 
							break;
						}
						else { inside = false; }
					}
				} while (inside);
				
				if (goal != null) { goal = null; }
				if (base != null) { base.clear(); }
				path.clear();
			}
		});
    	vfp.add( rtl );
    	
		vfp.add(x_target.getSliderControls(false));
		vfp.add(y_target.getSliderControls(false));
		vfp.add(z_target.getSliderControls(false));
		
		// Add a button to give a random starting position
		JButton sp = new JButton("Random Starting Position");
    	sp.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				
				boolean collision = false;
				do {
					for (int i = 0; i < kfs.dofList.size() - 1; i++) {
						kfs.dofList.get(i).setValue(rand.nextInt(360) - 180);
					}
					
					kfs.dofList.get(kfs.dofList.size()-1).setValue((rand.nextDouble() - 0.5) * hE);
					
					// Make sure the new starting position doesn't collide with any existing obstacles
					setDofs();
					collision = collideAll(dofs[0], dofs[1], dofs[2], dofs[3]);
					
				} while (collision);
			}
		});
    	vfp.add( sp );	
    	
    	// Sets the position of the robot to its current position
    	JButton set = new JButton("Set Position");
    	set.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {				
				setPDofs();
			}
		});
    	vfp.add( set );
    	
    	// Returns the robot to the set position
    	JButton init = new JButton("Initial Position");
    	init.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				
				if (pdofs != null) { 
					for (int i = 0; i < kfs.dofList.size(); i++) {
			        	kfs.dofList.get(i).setValue(pdofs[i]);
			        }
				}
			}
		});
    	vfp.add( init );
		
		vfp.add( kfs.getControls() );
    	
		// Adds a randomly placed obstacle of the given size
    	JButton a = new JButton("Add Obstacle");
    	a.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				
				setDofs();
				boolean collision = false;
				Vector3d l = new Vector3d();
				do {
					do {
						l.x = rand.nextDouble() * 2.0 * rad - rad;
						l.y = rand.nextDouble() * (rad - plane) - plane;
						l.z = rand.nextDouble() * 2.0 * rad - rad;
					} while (l.length() > rad);
					
					// Make sure the new obstacle doesn't collide with the robot
					objects.add(new Collidable(l, sphereSize.getFloatValue()));
					collision = collideAll(dofs[0], dofs[1], dofs[2], dofs[3]);
					if (collision) { objects.remove(objects.size() - 1); }				
				} while (collision);
			}
		});
    	vfp.add( a );
    	
    	// Randomizes the locations of all the existing obstacles
    	JButton rmize = new JButton("Randomize Obstacles");
    	rmize.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				setDofs();
				
				// Copy the collidables
				List<Collidable> temp = new LinkedList<Collidable>();
				for (int i = 0; i < objects.size(); i++) {
					temp.add(objects.get(i));
				}
				
				objects.clear();
				
				// Put them back into objects at random locations
				boolean collision = false;
				Vector3d l = new Vector3d();				
				for (int i = 0; i < temp.size(); i++) {
					do {
						do {
							l.x = rand.nextDouble() * 2.0 * rad - rad;
							l.y = rand.nextDouble() * (rad - plane) - plane;
							l.z = rand.nextDouble() * 2.0 * rad - rad;
						} while (l.length() > rad);
						
						// Make sure those locations don't collide with the robot
						objects.add(new Collidable(l, temp.get(i).radius));
						collision = collideAll(dofs[0], dofs[1], dofs[2], dofs[3]);
						if (collision) { objects.remove(objects.size() - 1); }
					} while (collision);
				}
			}
		});
    	vfp.add( rmize );
    	vfp.add(sphereSize.getSliderControls(false));
		vfp.add(epsilon.getSliderControls(false));
    	
		// Remove all the obstacles
    	JButton co = new JButton("Clear Obstacles");
    	co.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {				
				objects.clear();
			}
		});
    	vfp.add( co );
		
		vfp.add(t1_target.getSliderControls(false));
		vfp.add(t2_target.getSliderControls(false));
		vfp.add(t3_target.getSliderControls(false));
		vfp.add(l_target.getSliderControls(false));
		
		// A button to create the tree
		JButton ctree = new JButton("Create Tree (Method 1)");
    	ctree.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				createTree(1);
			}
		});
    	vfp.add( ctree );   	
    	vfp.add(threshold.getSliderControls(false));
    	
    	JButton ctree2 = new JButton("Create Tree (Method 2)");
    	ctree2.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				createTree(2);
			}
		});
    	vfp.add( ctree2 );
    	vfp.add(show_tree.getControls());		
    	return vfp.getPanel();
	}

	// Variables for the display function
    static final public GLUT glut = new GLUT();
    static final public float[] red = {1, 0, 0, 1};
    static final public float[] blue = {0, 0, 1, 1};
    static final public float[] green = {0, 0.75f, 0, 1};
    static final public float[] orange = {1, 0.75f, 0, 1};
	
    // Display the tree
	public void display( GLAutoDrawable drawable ) {
		
		// First display the key-framed scene
		kfs.display(drawable);		
        GL2 gl = drawable.getGL().getGL2();

        // Then draw a small sphere at the tip of the end-effector
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, red, 0 );
        setDofs();
        Vector3d c_location = findLocation(dofs[0], dofs[1], dofs[2], dofs[3]);
		gl.glPushMatrix();
		gl.glTranslated(c_location.x, c_location.y, c_location.z);
        glut.glutSolidSphere(rE, 20, 20);
        gl.glPopMatrix();  
        
        // Display the tree
        if (base != null && show_tree.getValue()) { base.display(drawable); }
        
        // Display the target location
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, blue, 0 );
        gl.glPushMatrix();
		gl.glTranslated(x_target.getFloatValue(), y_target.getFloatValue(), z_target.getFloatValue());
        glut.glutSolidSphere(rA, 20, 20);
        gl.glPopMatrix();     
        
        // Display the goal node
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, orange, 0 );
        if (goal != null){
	        gl.glPushMatrix();
			gl.glTranslated(goal.location.x, goal.location.y, goal.location.z);
	        glut.glutSolidSphere(rA, 20, 20);
	        gl.glPopMatrix();
        }
        
        // Draw all the obstacles
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, green, 0 );
        for (Collidable c : objects) {
        	c.display(drawable);
        }
	}
}
