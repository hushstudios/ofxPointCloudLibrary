#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxPointCloudLibrary.h"

class ofApp : public ofBaseApp
{

public:
	void setup();
	void update();
	void draw();
	void alignIcp();

	void keyPressed( int key );
	void keyReleased( int key );
	void mouseMoved( int x, int y );
	void mouseDragged( int x, int y, int button );
	void mousePressed( int x, int y, int button );
	void mouseReleased( int x, int y, int button );
	void mouseEntered( int x, int y );
	void mouseExited( int x, int y );
	void windowResized( int w, int h );
	void dragEvent( ofDragInfo dragInfo );
	void gotMessage( ofMessage msg );

	ofMesh mesh;
	of3dPrimitive pointsA, pointsB, pointsAligned;
	ofxPcl::Alignment alignment;  // iterative closest point alignment
	glm::mat4 alignMat = glm::mat4( 1. );

	ofEasyCam camera;

	ofxPanel gui;
	ofxButton alignBtn;                 // perform alignment
	ofParameter<bool> showUnaligned;	// toggle draw unaligned point cloud A
	ofParameter<bool> showAligned;		// toggle draw aligned point cloud A
	ofParameter<float> scale;           // scale of mesh
	ofParameter<glm::vec3> translateA;  // translation for point cloud A
	ofParameter<glm::vec3> rotateA;     // euler rotation for point cloud A
};
