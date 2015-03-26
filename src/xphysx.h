#include "ofMain.h"
#include <PxPhysicsAPI.h> 

using namespace std;
using namespace physx;

#pragma comment(lib, "PhysX3_x64.lib")
#pragma comment(lib, "PxTask.lib")
#pragma comment(lib, "Foundation.lib")
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "GeomUtils.lib") 

PxPhysics* gPhysicsSDK = NULL;
PxDefaultErrorCallback gDefaultErrorCallback;
PxDefaultAllocator gDefaultAllocatorCallback;
PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;
PxScene* gScene = NULL;
PxReal myTimestep = 1.0f/60.0f;
vector<PxRigidActor*> boxes;
PxDistanceJoint *gMouseJoint = NULL;
PxRigidDynamic* gMouseSphere = NULL;
PxReal gMouseDepth = 0.0f;
PxRigidDynamic* gSelectedActor=NULL;

int oldX=0, oldY=0;
float rX=15, rY=0;
float fps=0;
int startTime=0;
int totalFrames=0;
int state =1 ;
float dist=-23;

struct Ray {
   PxVec3 orig, dir;
};

GLdouble modelMatrix[16];
GLdouble projMatrix[16];
GLint viewPort[4];

void ViewProject(PxVec3 v, int &xi, int &yi, float &depth)
{  
	GLdouble winX, winY, winZ;
	gluProject((GLdouble) v.x, (GLdouble) v.y, (GLdouble) v.z, modelMatrix, projMatrix, viewPort, &winX, &winY, &winZ);
	xi = (int)winX; yi = viewPort[3] - (int)winY - 1; depth = (float)winZ;
 
}

void ViewUnProject(int xi, int yi, float depth, PxVec3 &v)
{
 	yi = viewPort[3] - yi - 1;
	GLdouble wx, wy, wz;
	gluUnProject((GLdouble) xi, (GLdouble) yi, (GLdouble) depth,
	modelMatrix, projMatrix, viewPort, &wx, &wy, &wz);
	v=PxVec3((PxReal)wx, (PxReal)wy, (PxReal)wz);
}

void LetGoActor()
{
	if (gMouseJoint) 
		gMouseJoint->release();
	gMouseJoint = NULL;
	if (gMouseSphere)
		 gMouseSphere->release();
	gMouseSphere = NULL;
}

PxRigidDynamic* CreateSphere(const PxVec3& pos, const PxReal radius, const PxReal density)
{
	PxTransform transform(pos, PxQuat::createIdentity());
	PxSphereGeometry geometry(radius);   
	PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5,0.5,0.5);
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0,0,0)); 
	gScene->addActor(*actor);
	return actor;
} 

void MoveActor(int x, int y)
{
	if (!gMouseSphere) 
		return;
	PxVec3 pos;
	ViewUnProject(x,y, gMouseDepth, pos);
	gMouseSphere->setGlobalPose(PxTransform( pos)); 
}

bool PickActor(int x, int y)
{
 	LetGoActor();

	Ray ray; 
	ViewUnProject(x,y,0.0f, ray.orig);
	ViewUnProject(x,y,1.0f, ray.dir);
	ray.dir -= ray.orig; 
	float length = ray.dir.magnitude();
	ray.dir.normalize();

	PxRaycastHit hit;
	PxShape* closestShape;
	gScene->raycastSingle(ray.orig, ray.dir, length, PxSceneQueryFlag::eIMPACT  ,hit);
	closestShape = hit.shape;
	if (!closestShape) return false;
	if (!closestShape->getActor())/*.is(PxActorType::eRIGID_DYNAMIC))*/ return false;
	int hitx, hity;
	ViewProject(PxVec3(hit.v), hitx, hity, gMouseDepth);
	gMouseSphere = CreateSphere(PxVec3(hit.v), 0.1f, 1.0f);

	gMouseSphere->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true); //PX_BF_KINEMATIC);
	  
	gSelectedActor = (PxRigidDynamic*) closestShape->getActor();
	gSelectedActor->wakeUp(); 

        PxTransform mFrame, sFrame;
 	mFrame.q = gMouseSphere->getGlobalPose().q;
        //mFrame.p = gMouseSphere->getGlobalPose().transformInv(hit.v );
	sFrame.q = gSelectedActor->getGlobalPose().q;
	//sFrame.p = gSelectedActor->getGlobalPose().transformInv(hit.v );
	
	gMouseJoint = PxDistanceJointCreate(*gPhysicsSDK, gMouseSphere, mFrame, gSelectedActor, sFrame);
	gMouseJoint->setDamping(1);
//	gMouseJoint->setStiffness(200);
	gMouseJoint->setMinDistance(0);
	gMouseJoint->setMaxDistance(0);
	gMouseJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
	gMouseJoint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
	return true;
}


void stepPX() 
{ 
	gScene->simulate(myTimestep);        
	while(!gScene->fetchResults() )     
	{
		// do something useful        
	}
} 

void initPX() {
	PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	if(!foundation)
	        ofLog(OF_LOG_ERROR)<<"PxCreateFoundation failed!"<<endl;
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale() );

	if(gPhysicsSDK == NULL) {
		cerr<<"Error creating PhysX3 device."<<endl;
		cerr<<"Exiting..."<<endl;
		exit(1);
	}
	 
        if(!PxInitExtensions(*gPhysicsSDK))
		cerr<< "PxInitExtensions failed!" <<endl;

	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
	sceneDesc.gravity=PxVec3(0.0f, -9.8f, 0.0f);

     if(!sceneDesc.cpuDispatcher) {
        PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
        if(!mCpuDispatcher)
           cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
        sceneDesc.cpuDispatcher = mCpuDispatcher;
    } 
 	if(!sceneDesc.filterShader)
        sceneDesc.filterShader  = gDefaultFilterShader;

	 
	gScene = gPhysicsSDK->createScene(sceneDesc);
	if (!gScene)
        cerr<<"createScene failed!"<<endl;

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				 1.0);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);

	
	PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5,0.5,0.5);
	 
    PxReal d = 0.0f;	 
	PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));

	PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
			cerr<<"create plane failed!"<<endl;

	PxShape* shape = plane->createShape(PxPlaneGeometry(), *mMaterial);
	if (!shape)
		cerr<<"create shape failed!"<<endl;
	gScene->addActor(*plane);


	PxReal density = 1.0f;
	PxTransform transform(PxVec3(0.0f, 5.0, 0.0f), PxQuat::createIdentity());
	PxVec3 dimensions(0.5,0.5,0.5);
	PxBoxGeometry geometry(dimensions);    
	
	for(int i=0;i<10;i++) {
	   transform.p  = PxVec3(0.0f,5.0f+5*i,0.0f);
	   PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	   if (!actor)
		cerr<<"create actor failed!"<<endl;
       actor->setAngularDamping(0.75);
       actor->setLinearVelocity(PxVec3(0,0,0)); 
	   gScene->addActor(*actor);
	   boxes.push_back(actor);
	}
	
}
 
void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
   mat[0] = m.column0[0];
   mat[1] = m.column0[1];
   mat[2] = m.column0[2];
   mat[3] = 0;

   mat[4] = m.column1[0];
   mat[5] = m.column1[1];
   mat[6] = m.column1[2];
   mat[7] = 0;

   mat[8] = m.column2[0];
   mat[9] = m.column2[1];
   mat[10] = m.column2[2];
   mat[11] = 0;

   mat[12] = t[0];
   mat[13] = t[1];
   mat[14] = t[2];
   mat[15] = 1;
}
 
void DrawBox(PxShape* pShape, PxRigidActor * actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape,*actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
    float mat[16];
	getColumnMajor(m,pT.p, mat);
	glPushMatrix(); 
		glMultMatrixf(mat);
		ofDrawBox(bg.halfExtents.x*2);
    glPopMatrix(); 
}

void DrawSphere(PxShape* pShape, PxRigidActor * actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape,*actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
	float mat[16];
	getColumnMajor(m,pT.p, mat);
	glPushMatrix(); 
		glMultMatrixf(mat);
	        ofDrawSphere(10);
	glPopMatrix(); 
}

void DrawShape(PxShape* shape,PxRigidActor *actor) 
{ 
	PxGeometryType::Enum type = shape->getGeometryType();
    switch(type) 
    {          
		case PxGeometryType::eBOX:
			glColor3f(0.5,0.5,0.5);
			DrawBox(shape,actor);
		break;

		case PxGeometryType::eSPHERE :
			glColor3f(0,1,0);
			DrawSphere(shape,actor);
		break;
    } 
} 

void DrawActor(PxRigidActor* actor) 
{  
	PxU32 nShapes = actor->getNbShapes(); 
    PxShape** shapes=new PxShape*[nShapes];
	
	actor->getShapes(shapes, nShapes);     
    while (nShapes--) 
    { 
        DrawShape(shapes[nShapes],actor); 
    } 
	delete [] shapes;
} 

void RenderActors()  { 
	for(int i=0;i<boxes.size();i++ ) {
		DrawActor(boxes[i]);
	}

	if(gMouseSphere) {
		DrawActor(gMouseSphere);
	}
} 
 



void shutdownPX() {
	for(int i=0;i<boxes.size();i++) {
		gScene->removeActor(*boxes[i]);	
		boxes[i]->release();
	}

	boxes.clear();
	gScene->release();
	
	gPhysicsSDK->release();
}

void render() {
    if (gScene) 
    { 
        stepPX(); 
    } 


	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	ofDrawGrid(10.0f,8.0f,false,false,true,false);
	
}

void MouseDown(int x, int y) {
	PickActor(x,y);
}

void MouseUP(){
  	LetGoActor();
}

void Motion(int x, int y) {
	MoveActor(x,y);
}
