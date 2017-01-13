package project;

import java.util.Collection;
import java.util.LinkedList;

import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

import com.jogamp.opengl.util.gl2.GLUT;

// Base class for scene nodes
public abstract class DAGNode {
    
	String name = "name";
	
	// Children for each node
    LinkedList<DAGNode> children = new LinkedList<DAGNode>();

    // A list of all degrees of freedom
    Collection<DoubleParameter> dofs = new LinkedList<DoubleParameter>();
        
	public boolean show = true;
            
    static final public GLUT glut = new GLUT();
    
    public void add( DAGNode n ) {
    	children.add( n );
    }
    
    // Recursively creates the controls for the DOFs of the nodes
    public JPanel getControls() {
    	if ( dofs.isEmpty() && children.isEmpty() ) return null;
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
    	if ( show == true ) { vfp.setBorder( new TitledBorder(name) ); }
    	for ( DoubleParameter p : dofs ) {
    		vfp.add( p.getSliderControls(false) );
    	}
    	for ( DAGNode n : children ) {
    		JPanel p = n.getControls();
    		if ( p != null) {
    			vfp.add( p );
    		}
    	}
    	CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel() );
    	return cp;
    }
    
    // Recursively collects all the DOFs for use in creating key poses
    public void getDOFs( Collection<DoubleParameter> dofs ) {
    	dofs.addAll( this.dofs );
    	for ( DAGNode n : children ) {
			n.getDOFs(dofs);
		}
    }
    
    // Draws the node and all its children
    public void display( GLAutoDrawable drawable ) {    	
		for ( DAGNode n : children ) {
			n.display(drawable);
		}
    }  
}
