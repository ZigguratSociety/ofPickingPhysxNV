#include "ofMain.h"
#include "ofAppGlutWindow.h"
#include "xphysx.h"

class xApp : public ofBaseApp {

	public:
		ofEasyCam cam;
		int w,h;
		bool start;

		void setup(){
			ofSetFrameRate(150);
			w=ofGetScreenWidth();
			h=ofGetScreenHeight();
			initPX();
		}

		void exit(){
		        shutdownPX();
		}

		void update(){
			ofSetWindowTitle(ofToString(ofGetFrameRate()));
		}

		void draw(){
			ofBackgroundGradient(255,0);

			cam.begin();
				render();
			cam.end();
		}

		void keyPressed(int key){

		}

		void mouseMoved(int x, int y){
			Motion(x,y);
		}
		void mousePressed(int x, int y, int button){
			MouseDown(x,y);
		}
		void mouseReleased(int x, int y, int button){
			MouseUP();
		}
};

int main( ){
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1280, 720, OF_WINDOW);
	ofRunApp(new xApp());
}
