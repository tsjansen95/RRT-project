package project;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JPanel;

import mintools.swing.ControlFrame;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.TrackBallCamera;

import com.jogamp.opengl.DebugGL2;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.util.FPSAnimator;

public class ProjectApp implements GLEventListener {

	// Create the tree
	public RRT tree = new RRT();
	
	// Put name in the window title
	private String name = "RRT Project - THOMAS JANSEN";    
        
    // Launches the application
    public static void main(String[] args) {
        new ProjectApp();
    }
    
    public GLCanvas glCanvas;
    
    // Main trackball for viewing the world
    public TrackBallCamera tbc = new TrackBallCamera();
        
    // Creates the application
    public ProjectApp() {      
        Dimension controlSize = new Dimension(640, 640);
        Dimension size = new Dimension(640, 480);
        ControlFrame controlFrame = new ControlFrame("Controls");
        controlFrame.add("Scene", getControls());
        controlFrame.setSize(controlSize.width, controlSize.height);
        controlFrame.setLocation(size.width + 20, 0);
        controlFrame.setVisible(true);    
        GLProfile glp = GLProfile.getDefault();
        GLCapabilities glc = new GLCapabilities(glp);
        glCanvas = new GLCanvas( glc );
        glCanvas.setSize( size.width, size.height );
        glCanvas.setIgnoreRepaint( true );
        glCanvas.addGLEventListener( this );
        glCanvas.requestFocus();
        FPSAnimator animator = new FPSAnimator( glCanvas, 60 );
        animator.start();        
        tbc.attach( glCanvas );
        JFrame frame = new JFrame( name );
        frame.getContentPane().setLayout( new BorderLayout() );
        frame.getContentPane().add( glCanvas, BorderLayout.CENTER );
        frame.setLocation(0,0);        
        frame.addWindowListener( new WindowAdapter() {
            @Override
            public void windowClosing( WindowEvent e ) {
                System.exit(0);
            }
        });
        frame.pack();
        frame.setVisible( true );        
    }
    
    @Override
    public void dispose(GLAutoDrawable drawable) {
    	// nothing to do
    }
        
    @Override
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
        // do nothing, glViewport already handled by the component
    }
    
    // Create the control panel
    public JPanel getControls() {     
        VerticalFlowPanel vfp = new VerticalFlowPanel();              
    	vfp.add( tree.getControls() );
        return vfp.getPanel();
    }
    
    // Initialize opengl
    public void init( GLAutoDrawable drawable ) {
    	drawable.setGL( new DebugGL2( drawable.getGL().getGL2() ) );
        GL2 gl = drawable.getGL().getGL2();
        gl.glShadeModel(GL2.GL_SMOOTH);             // Enable Smooth Shading
        gl.glClearColor(0.0f, 0.0f, 0.0f, 0.5f);    // Black Background
        gl.glClearDepth(1.0f);                      // Depth Buffer Setup
        gl.glEnable(GL.GL_BLEND);
        gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);
        gl.glEnable(GL.GL_LINE_SMOOTH);
        gl.glEnable(GL2.GL_POINT_SMOOTH);
        gl.glEnable(GL2.GL_NORMALIZE );
        gl.glEnable(GL.GL_DEPTH_TEST);              // Enables Depth Testing
        gl.glDepthFunc(GL.GL_LEQUAL);               // The Type Of Depth Testing To Do 
        gl.glLineWidth( 2 );                        // slightly fatter lines by default!
    }   
    
    // Light properties
    private final float[] lightPos = { 0, 100, 0 }; 
	private final float[] white = {1,1,1,1};
	private final float[] grey = {0.75f,0.75f,0.75f,1f};
	private final float[] black = {0,0,0,1};
	private final float[] yellow = {1,1,0,1};

	// Sets the lights
    private void setLights( GLAutoDrawable drawable ) {
    	GL2 gl = drawable.getGL().getGL2();
		gl.glEnable( GL2.GL_LIGHTING );
		gl.glEnable( GL2.GL_LIGHT0 );
		// WATCH OUT: need to provide homogeneous coordinates to many calls!! 
		float[] lightPosition = {lightPos[0], lightPos[1], lightPos[2], 1}; 
		float[] dark = new float[] {0.1f,0.1f,0.1f,1};
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_POSITION, lightPosition, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_DIFFUSE, white, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_AMBIENT, black, 0);
		gl.glLightModelfv( GL2.GL_LIGHT_MODEL_AMBIENT, dark, 0);
    }
    
    private int list = -1;
    
    // Display the scene
    @Override
    public void display(GLAutoDrawable drawable) {        
        GL2 gl = drawable.getGL().getGL2();
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);            
			
        // We will use a trackball camera 
        tbc.prepareForDisplay(drawable);
        gl.glScaled( 15, 15, 15 );
    		        
        gl.glPushMatrix();
        gl.glScaled( 0.1,0.1,0.1 );        
        setLights( drawable );
                
        // Create the plane
        if ( list != -1 ) {
        	gl.glCallList(list);
        } else {
        	list = gl.glGenLists(1);
        	gl.glNewList(list, GL2.GL_COMPILE_AND_EXECUTE );        
	        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_SPECULAR, white, 0 );
	        gl.glMaterialf( GL.GL_FRONT_AND_BACK, GL2.GL_SHININESS, 127 );
	        for ( int i = -20; i < 20; i++ ) {
	        	for ( int j = -20; j <= 10; j++ ) {
	                gl.glBegin( GL2.GL_QUAD_STRIP );
	                gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, ((i+j)%2)==0?grey:white, 0 );
	                gl.glNormal3f(0,1,0);
	                gl.glVertex3d( i, -1, j );
	                gl.glVertex3d( i, -1, j+1 );
	                gl.glVertex3d( i+1, -1, j );
	                gl.glVertex3d( i+1, -1, j+1 );        
	                gl.glEnd();
	        	}
	        }
	    	gl.glEndList();
        }        
        gl.glPopMatrix();

        // Display the tree
        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, yellow, 0 );
        tree.display(drawable);
              
    }
    
}
