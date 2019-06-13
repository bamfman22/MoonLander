
//--------------------------------------------------------------
//
//  Kevin M. Smith 
//
//  Mars HiRise Project - startup scene
// 
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have reconstructed from Mars
//  HiRis photographs taken the Mars Reconnaisance Orbiter
//
//  You will use this source file (and include file) as a starting point
//  to implement assignment 5  (Parts I and II)
//
//  Please do not modify any of the keymappings.  I would like 
//  the input interface to be the same for each student's 
//  work.  Please also add your name/date below.

//  Please document/comment all of your work !
//  Have Fun !!
//
//  Student Name:   < Derek Ortega >
//  Date: <4/22/19>


#include "ofApp.h"
#include "Util.h"
#include "vector3.h"


//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bRoverLoaded = false;
	bTerrainSelected = true;
    intersected = false;
    
    gui.setup();
    gui.add(labelAltitude.setup("Altitude", "0"));
//	ofSetWindowShape(1024, 768);
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	ofSetVerticalSync(true);
	cam.disableMouseInput();
    
    
    backgroundSound.load("sounds/space.mp3");
    exhaustSound.load("sounds/exhaust.mp3");
    pCam = &cam;
    
    backgroundSound.setLoop(1);
    backgroundSound.play();
    background.load("background.jpeg");
    background.resize(1280, 1024);
	ofEnableSmoothing();
	ofEnableDepthTest();

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

    if(lander.loadModel("geo2/lander.obj")){
        lander.setScaleNormalization(false);
        lander.setScale(0.5,0.5,0.5);
        lander.setRotation(0,-180,1,0,0);
        lander.setPosition(0,10,0);
        bRoverLoaded = true;
    }
    else {
        cout << "Error: Can't load model" << "geo/lander.obj" << endl;
        ofExit(0);
    }
    
    landerBox = meshBounds(lander.getMesh(0));
    
    if(terrain.loadModel("geo2/mars-low.obj")){
        terrain.setScaleNormalization(false);
        terrain.setScale(1, 1, 1);
        terrain.setRotation(0, -180, 1, 0, 0);
        
    }

    meshTerrain = terrain.getMesh(0);
    
	boundingBox = meshBounds(terrain.getMesh(0));
    
    octree.create(meshTerrain, 8);
    
    
    lem = ParticleEmitter(new ParticleSystem());
    lem.setPosition(lander.getPosition());
    lem.setLifespan(50);
    lem.setVelocity(ofVec3f(0,0,0));
    lem.sys->addForce(new TurbulenceForce(ofVec3f(-.1,-.3,-.1), ofVec3f(.2,.2,.1)));
    
    
    exhaustParticles = ParticleEmitter(new ParticleSystem());
    exhaustParticles.setPosition(lander.getPosition());
    exhaustParticles.sys->addForce(new ImpulseRadialForce(200));
    exhaustParticles.setEmitterType(DiscEmitter);
    exhaustParticles.setGroupSize(100);
    exhaustParticles.start();
    
	
    
    
    topCam.setNearClip(.1);
    topCam.setFov(65.5);
    topCam.setPosition(0,10,0);
    topCam.lookAt(glm::vec3(0,0,0));
    
    trackingCam.setNearClip(.1);
    trackingCam.setFov(65.5);
    trackingCam.setPosition(10,10,10);
    trackingCam.lookAt(lander.getPosition());
    
    frontCam.setNearClip(2);
    frontCam.setFov(65.5);
    frontCam.setPosition(glm::vec3(lander.getPosition().x,lander.getPosition().y,lander.getPosition().z));
    frontCam.lookAt(glm::vec3(frontCam.getPosition().x, frontCam.getPosition().y, frontCam.getPosition().z - 10));
    
    downCam.setNearClip(.1);
    downCam.setFov(65.5);
    downCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y - .75, lander.getPosition().z));
    downCam.lookAt(glm::vec3(downCam.getPosition().x, downCam.getPosition().y - 1, downCam.getPosition().z));
}


bool ofApp::collisionDetect(){
    ofPoint landerPosition = lander.getPosition();
    
    Vector3 x = Vector3(landerPosition.x, landerPosition.y + landerBox.height() / 2.f, landerPosition.z);
    
    Ray ray(x, Vector3(0, -1,0));
    ofVec3f rayDir(0,-1,0);
    
    TreeNode hit;
    ofPoint terrainPosition = terrain.getPosition();
    
    if(octree.intersect(ray, octree.root, hit)){
        float widthX = landerBox.widthX() * 0.5;
        float widthZ = landerBox.widthZ() * 0.5;
        for(int i = 0; i< hit.points.size(); i++){
            ofVec3f v = meshTerrain.getVertex(hit.points[i]) + terrainPosition;
            if(v.x >= x.x() - widthX / 2 && v.x <= x.x() + widthX/2 && v.z >= x.z() - widthZ / 2 && v.z <= x.z() + widthZ / 2){
                ofVec3f dir = v - landerPosition;
                if(x.y() - v.y <= 0){
                    altitude = 0;
                    return true;
                }
                else{
                    altitude = x.y() - v.y;
                }
            }
        }
        return false;
    }
    
}
//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
    
    if(collisionDetect()){
        return;
    }
    if(lem.sys->particles.size() > 0){
        lem.stop();
    }
    
    if(lem.sys->particles.size()<1){
        lem.start();
    }
    
    for(int i = 0; i< lem.sys->particles.size(); i++){
        lander.setPosition(lem.sys->particles[i].position.x,lem.sys->particles[i].position.y, lem.sys->particles[i].position.z);
        lem.setPosition(lander.getPosition());
        exhaustParticles.setPosition(lander.getPosition());
        
    }
    lem.update();
    lander.update();
    
    if(exhaustOn){
        exhaustParticles.setLifespan(.1);
        exhaustParticles.setRate(10);
        exhaustParticles.setVelocity(ofVec3f(0,-20,0));
        exhaustParticles.start();
        exhaustSound.play();
    }
    
    else{
        exhaustParticles.setRate(0);
    }
    exhaustParticles.update();
    
    
    frontCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z));
    frontCam.lookAt(glm::vec3(frontCam.getPosition().x, frontCam.getPosition().y, frontCam.getPosition().z - 10));
    downCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y-75, lander.getPosition().z));
    downCam.lookAt(glm::vec3(downCam.getPosition().x, downCam.getPosition().y - 1, downCam.getPosition().z));
    topCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y + 10, lander.getPosition().z));
    topCam.lookAt(lander.getPosition());
    trackingCam.lookAt(lander.getPosition());
}
//--------------------------------------------------------------
void ofApp::draw(){



    ofPushMatrix();
    ofDisableDepthTest();
    background.draw(0, 0);
    ofEnableDepthTest();
    
    
	pCam->begin();
	ofPushMatrix();
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		terrain.drawWireframe();
		if (bRoverLoaded) {
			lander.drawWireframe();
			
		}
        
		
	}
	else {
		ofEnableLighting();              // shaded mode
		terrain.drawFaces();

		if (bRoverLoaded) {
			lander.drawFaces();
			
		}
		
	}

    ofDrawSphere(glm::vec3(0,0,0), 1);
	

   
    exhaustParticles.draw();

	
	
	
  
    

	

	ofPopMatrix();
	pCam->end();
    
    labelAltitude = ofToString(altitude);
    gui.draw();
    
    string str;
    str = "Altitude: " + ofToString(altitude);
    ofDrawBitmapString(str, ofGetWindowWidth() - 170, 35);
}

// 

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
	case 'r':
		cam.reset();
		break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
    case 'w':
    case 'W':
        exhaustOn=true;
        lem.sys->addForce(new ThrustForce(ofVec3f(0,.1,0)));
        break;
    case 'a':
    case 'A':
        exhaustOn = true;
        lem.sys->addForce(new ThrustForce(ofVec3f(.1,0,0)));
        break;
    case 's':
    case 'S':
        exhaustOn = true;
        lem.sys->addForce(new ThrustForce(ofVec3f(0,-.1,0)));
        break;
    case 'd':
    case 'D':
        exhaustOn = true;
        lem.sys->addForce(new ThrustForce(ofVec3f(-.1,0,0)));
        break;
        case OF_KEY_F1:
            pCam = &cam;
            break;
        case OF_KEY_F2:
            pCam = &frontCam;
            break;
        case OF_KEY_F3:
            pCam = &downCam;
            break;
        case OF_KEY_F4:
            pCam = &topCam;
            break;
        case OF_KEY_F5:
            pCam = &trackingCam;
            break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

    switch (key) {
            
        case OF_KEY_ALT:
            cam.disableMouseInput();
            bAltKeyDown = false;
            break;
        case OF_KEY_CONTROL:
            bCtrlKeyDown = false;
            break;
        case OF_KEY_SHIFT:
            break;
        case 'w':
            exhaustOn = false;
            lem.sys->reset();
            break;
        case 's':
            exhaustOn = false;
            lem.sys->reset();
            break;
        case 'd':
            exhaustOn = false;
            lem.sys->reset();
            break;
        case 'a':
            exhaustOn = false;
            lem.sys->reset();
            break;
        default:
            break;
            
    }
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));
	
    int start = ofGetElapsedTimeMillis();
    if(octree.intersect(ray,octree.root,octree.currentNode)){
        intersected = true;
        int point;
        point = octree.currentNode.points[0];
        intersectPoint = terrain.getMesh(0).getVertex(point);
        int end = ofGetElapsedTimeMillis();
        
        cout << end- start << "Milliseconds to select the node" << endl << endl;
    }
}


//draw a box from a "Box" class  
//
void ofApp::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void ofApp::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {


}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}


//
//  ScreenSpace Selection Method: 
//  This is not the octree method, but will give you an idea of comparison
//  of speed between octree and screenspace.
//
//  Select Target Point on Terrain by comparing distance of mouse to 
//  vertice points projected onto screenspace.
//  if a point is selected, return true, else return false;
//
bool ofApp::doPointSelection() {

	ofMesh mesh = mars.getMesh(0);
	int n = mesh.getNumVertices();
	float nearestDistance = 0;
	int nearestIndex = 0;

	bPointSelected = false;

	ofVec2f mouse(mouseX, mouseY);
	vector<ofVec3f> selection;

	// We check through the mesh vertices to see which ones
	// are "close" to the mouse point in screen space.  If we find 
	// points that are close, we store them in a vector (dynamic array)
	//
	for (int i = 0; i < n; i++) {
		ofVec3f vert = mesh.getVertex(i);
		ofVec3f posScreen = cam.worldToScreen(vert);
		float distance = posScreen.distance(mouse);
		if (distance < selectionRange) {
			selection.push_back(vert);
			bPointSelected = true;
		}
	}

	//  if we found selected points, we need to determine which
	//  one is closest to the eye (camera). That one is our selected target.
	//
	if (bPointSelected) {
		float distance = 0;
		for (int i = 0; i < selection.size(); i++) {
			ofVec3f point =  cam.worldToCamera(selection[i]);

			// In camera space, the camera is at (0,0,0), so distance from 
			// the camera is simply the length of the point vector
			//
			float curDist = point.length(); 

			if (i == 0 || curDist < distance) {
				distance = curDist;
				selectedPoint = selection[i];
			}
		}
	}
	return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);

	if (lander.loadModel(dragInfo.files[0])) {
		lander.setScaleNormalization(false);
		lander.setScale(.005, .005, .005);
		lander.setPosition(point.x, point.y, point.z);
		bRoverLoaded = true;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	glm::vec3 mouse(mouseX, mouseY, 0);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}
