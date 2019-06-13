#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include "ofxGui.h"
#include "ParticleSystem.h"
#include "ParticleEmitter.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool  doPointSelection();
    bool collisionDetect();
		void drawBox(const Box &box);
		Box meshBounds(const ofMesh &);
		void subDivideBox8(const Box &b, vector<Box> & boxList);

		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);

		ofEasyCam cam;
		ofxAssimpModelLoader mars, lander, terrain;
		ofLight light;
		Box boundingBox, landerBox;
    float landerScale;
		vector<Box> level1, level2, level3;
	
        ofImage background;
    ofSoundPlayer backgroundSound;
    ofSoundPlayer exhaustSound;
    
    ParticleEmitter lem;
    ParticleEmitter exhaustParticles;
    
    ofMesh meshTerrain;
    
        bool exhaustOn;
		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
        bool intersected;
		bool bRoverLoaded;
		bool bTerrainSelected;

	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;
    
        Octree octree;

		const float selectionRange = 4.0;
    
    ofxPanel gui;
    ofxLabel labelAltitude;
    float altitude;
    
    
    ofCamera trackingCam;
    ofCamera frontCam;
    ofCamera downCam;
    ofCamera *pCam = NULL;
    ofCamera topCam;
};
