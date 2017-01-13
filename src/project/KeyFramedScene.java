package project;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;

public class KeyFramedScene {


    // 1800 frames at 30 frames per section for 1 minute of animation
    public final int num_frames = 2000;
    
    // The root of the robot
    private DAGNode root = null;
    
    // Master list of degress of freedom
    public List<DoubleParameter> dofList = new ArrayList<DoubleParameter>();

    // key poses are null when not set
    private ArrayList<double[]> keyPoses = new ArrayList<double[]>( num_frames );
    
    // Time line slide control for selecting the animation frame
    private JSlider keyFrameSlider = new JSlider( 0, num_frames - 1, 0 );

    private BooleanParameter animate = new BooleanParameter("animate", false );

    private JPanel vfpPosePanel; 
    
    private VerticalFlowPanel vfpPose = new VerticalFlowPanel();
        
	public KeyFramedScene() {
    	// initialize the key frames array to have no key frames
    	for ( int i = 0; i < num_frames; i++ ) {
    		keyPoses.add( null );
    	}
    	createCharacter();
	}
	
	// Display the appropriate frame
	public void display( GLAutoDrawable drawable ) {
        if ( animate.getValue() ) {
        	keyFrameSlider.setValue( (keyFrameSlider.getValue() + 1) % num_frames );
        } 
        if ( root != null ){
        	root.display(drawable);
        }
	}
	
	// Get the controls for the key frame animation
    public JPanel getControls() {
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
    	
    	HorizontalFlowPanel hfp = new HorizontalFlowPanel();
    	hfp.add( animate.getControls() );
    	vfp.add( hfp.getPanel() );
    	vfp.add( keyFrameSlider );
    	
    	keyFrameSlider.addChangeListener( new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				updatePose();
			}
		});
    		
    	vfpPose.setBorder( new TitledBorder("Pose") );
    	vfpPosePanel = vfpPose.getPanel();
    	vfp.add( vfpPosePanel );
    	return vfp.getPanel();
    }

    // Creates the character
    public void createCharacter() {
    	deleteAllKeyFrames();
        dofList.clear();
        vfpPose.removeAll();
        
        // Create the robot
    	root = RRT.create(0, -150, 120, 0.25);
    	
    	// Set the slider details
    	root.getDOFs( dofList );
    	int labelWidth = DoubleParameter.DEFAULT_SLIDER_LABEL_WIDTH;
    	int textWidth = DoubleParameter.DEFAULT_SLIDER_TEXT_WIDTH;
    	DoubleParameter.DEFAULT_SLIDER_LABEL_WIDTH = 50;
    	DoubleParameter.DEFAULT_SLIDER_TEXT_WIDTH = 50;
    	vfpPose.add( root.getControls() );    	
    	if (vfpPosePanel != null ) {
    		vfpPosePanel.updateUI();
    	}
    	DoubleParameter.DEFAULT_SLIDER_LABEL_WIDTH = labelWidth;
    	DoubleParameter.DEFAULT_SLIDER_TEXT_WIDTH = textWidth;
    }
    
    // Set the given key frame based on current state values
    public void setKeyFrame(int index) {
    	double[] key = keyPoses.get(index);
    	if ( key == null ) {
    		key = new double[dofList.size()];
    		keyPoses.set(index, key);
        	keyFrameSlider.updateUI();
    	}
        for ( int i = 0; i < dofList.size(); i++ ) {
        	key[i] = dofList.get(i).getValue();                
        }
    }
    
    // Delete all the key frames
    public void deleteAllKeyFrames() {
        for ( int i = 0; i < keyPoses.size(); i++ ) {
        	if ( keyPoses.get(i) != null ) {
        		keyPoses.set( i, null );
        	}
        }
    	keyFrameSlider.updateUI();
    }
    
    // Update the pose of the robot
    public void updatePose() {
    	int index = keyFrameSlider.getValue();
		if ( keyPoses.get(index) != null ) {
    		for ( int i = 0; i < dofList.size(); i++ ) {
            	dofList.get(i).setValue( keyPoses.get(index)[i] );                
            }        
    		return;
		}
		int prev = index-1;
		while ( prev >= 0 && keyPoses.get(prev) == null ) {
			prev--;
    	}
    	if ( prev >= 0 ) {
			int next = index+1;
			while ( next < num_frames && keyPoses.get(next) == null ) {
				next++;
	    	}
	    	if ( next < num_frames ) {
	    		double T = next - prev;
	    		double v = index - prev;
	    		double alpha = v / T;
	            for ( int i = 0; i < dofList.size(); i++ ) {
	            	DoubleParameter p = dofList.get(i);
	                double v1 = keyPoses.get(prev)[i];
	                double v2 = keyPoses.get(next)[i];
	                p.setValue( alpha*v2 + (1-alpha)*v1 );
	            }
	    	}
    	}
	}  
}
