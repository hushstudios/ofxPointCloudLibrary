#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{

	// we'll load a mesh and then create two copies with missing / noisy data ("points A" and "points B")

	mesh.load( "penguin.ply" );

	auto& verts         = mesh.getVertices();
	const auto& indices = mesh.getIndices();

	// fix model orientation
	glm::quat rot = glm::angleAxis( ofDegToRad( -90.f ), glm::vec3{ 1.f, 0.f, 0.f } );
	for ( auto& vert : mesh.getVertices() ) {
		vert = rot * vert;
	}

	// copy mesh to pointsA and pointsB and degrade
	auto& meshA = pointsA.getMesh();
	auto& meshB = pointsB.getMesh();

	for ( int i = 0; i < indices.size(); i += 3 ) {

		// add face to point cloud A
		if ( ofRandom( 0., 1. ) < 0.1 ) {
			auto id = meshA.getVertices().size();
			meshA.addVertex( verts[indices[i]] );
			meshA.addVertex( verts[indices[i + 1]] );
			meshA.addVertex( verts[indices[i + 2]] );
			meshA.addIndex( id );
			meshA.addIndex( id + 1 );
			meshA.addIndex( id + 2 );
		}

		// add face to point cloud B
		if ( ofRandom( 0., 1. ) < 0.1 ) {
			auto id = meshB.getVertices().size();
			meshB.addVertex( verts[indices[i]] );
			meshB.addVertex( verts[indices[i + 1]] );
			meshB.addVertex( verts[indices[i + 2]] );
			meshB.addIndex( id );
			meshB.addIndex( id + 1 );
			meshB.addIndex( id + 2 );
		}
	}

	ofLogNotice() << "Mesh A has: " << meshA.getVertices().size() << " verts, " << meshA.getIndices().size() / 3 << " faces";
	ofLogNotice() << "Mesh B has: " << meshB.getVertices().size() << " verts, " << meshB.getIndices().size() / 3 << " faces.";

	// gui
	gui.setup( "App Controls", "settings.json" );
	gui.add( alignBtn.setup( "perform alignment" ) );
	gui.add( showUnaligned.set( "show unaligned (pink)", true ) );
	gui.add( showAligned.set( "show aligned (green)", true ) );
	gui.add( scale.set( "mesh scale", 10., 0., 25. ) );
	gui.add( translateA.set( "translate points A", glm::vec3( 5., 10., 15. ), glm::vec3( -100. ), glm::vec3( 100 ) ) );
	gui.add( rotateA.set( "rotate points A", glm::vec3( 0., 25., 3. ), glm::vec3( -180 ), glm::vec3( 180 ) ) );

	alignBtn.addListener( this, &ofApp::alignIcp );

	camera.setDistance( 700. );
}

//--------------------------------------------------------------
void ofApp::update()
{
	pointsA.setPosition( translateA.get() );
	pointsA.setOrientation( rotateA.get() );

	// scale affects both
	pointsA.setScale( scale.get() );
	pointsB.setScale( scale.get() );
}

//--------------------------------------------------------------
void ofApp::draw()
{

	camera.begin();
	ofEnableDepthTest();

	ofSetColor( 255 );
	ofDrawAxis( 100 );

	// b is the target cloud
	ofSetColor( 127 );
	pointsB.draw();

	// a is the unaligned cloud
	if ( showUnaligned.get() ) {
		ofSetColor( 200, 0, 100 );
		pointsA.draw();
	}

	// aligned point cloud
	if ( showAligned.get() ) {
		ofSetColor( 0, 127, 0 );
		pointsAligned.draw();
	}

	ofDisableDepthTest();
	camera.end();

	ofSetColor( 255 );

	gui.draw();
}

//--------------------------------------------------------------
void ofApp::alignIcp()
{
	// build point clouds from meshes and transforms
	std::vector<glm::vec3> sourcePoints, targetPoints;

	// points A = source
	sourcePoints.reserve( pointsA.getMesh().getVertices().size() );
	for ( auto& vert : pointsA.getMesh().getVertices() ) {
		sourcePoints.push_back( pointsA.getGlobalTransformMatrix() * glm::vec4( vert, 1. ) );
	}

	// points B = target
	targetPoints.reserve( pointsB.getMesh().getVertices().size() );
	for ( auto& vert : pointsB.getMesh().getVertices() ) {
		targetPoints.push_back( pointsB.getGlobalTransformMatrix() * glm::vec4( vert, 1. ) );
	}

	float t0       = ofGetElapsedTimef();
	bool converged = alignment.align( sourcePoints, targetPoints );
	float t1       = ofGetElapsedTimef();

	alignMat = alignment.getAlignmentMatrix();

	ofLogNotice() << "Alignment took " << ofToString( ( t1 - t0 ) * 1000., 2. ) << " ms\n"
	              << "\tconverged: " << std::boolalpha << alignment.hasConverged() << "\n"
	              << "\ttransformation matrix (glm::mat4):\n"
	              << alignMat;

	// rebuild pointsAligned, points A -> points A transform -> alignment transform
	pointsAligned = pointsA;
	pointsAligned.resetTransform();

	// either manually apply the matrix to each vertex...
	//auto& alignedVerts = pointsAligned.getMesh().getVertices();
	//for ( auto& vert : alignedVerts ) {
	//	vert = alignMat * pointsA.getGlobalTransformMatrix() * glm::vec4( vert, 1. );
	//}

	// or decompose matrix, and apply the transformation to the ofNode
	glm::mat4 combinedMat = alignMat * pointsA.getGlobalTransformMatrix();
	glm::vec3 scale;
	glm::quat rotation;
	glm::vec3 translation;
	glm::vec3 skew;
	glm::vec4 perspective;
	ofxPcl::decomposeTransform( combinedMat, scale, rotation, translation, skew, perspective );
	pointsAligned.setPosition( translation );
	pointsAligned.setOrientation( rotation );
	pointsAligned.setScale( scale );
}

//--------------------------------------------------------------
void ofApp::keyPressed( int key )
{
}

//--------------------------------------------------------------
void ofApp::keyReleased( int key )
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved( int x, int y )
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged( int x, int y, int button )
{
}

//--------------------------------------------------------------
void ofApp::mousePressed( int x, int y, int button )
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased( int x, int y, int button )
{
}

//--------------------------------------------------------------
void ofApp::mouseEntered( int x, int y )
{
}

//--------------------------------------------------------------
void ofApp::mouseExited( int x, int y )
{
}

//--------------------------------------------------------------
void ofApp::windowResized( int w, int h )
{
}

//--------------------------------------------------------------
void ofApp::gotMessage( ofMessage msg )
{
}

//--------------------------------------------------------------
void ofApp::dragEvent( ofDragInfo dragInfo )
{
}
