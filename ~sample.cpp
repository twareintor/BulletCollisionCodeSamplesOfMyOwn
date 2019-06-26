// BulletCollisionCodeSamplesOfMyOwn
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <iostream>
#include <btBulletDynamicsCommon.h>
#include <irrlicht.h>
#include <windows.h>


#pragma comment(lib, "irrlicht.lib")


// #define CX_COMPILE_VS_DEBUG_MODE

#if !defined(CX_COMPILE_VS_DEBUG_MODE)
#pragma comment(lib, "BulletCollision_vs2010_x64_release.lib")
#pragma comment(lib, "BulletDynamics_vs2010_x64_release.lib")
// #pragma comment(lib, "BulletSoftBody_vs2010_x64_release.lib")
// #pragma comment(lib, "ConvexDecomposition_vs2010_x64_release.lib")
// #pragma comment(lib, "HACD_vs2010_x64_release.lib")
#pragma comment(lib, "LinearMath_vs2010_x64_release.lib")
#pragma comment(lib, "OpenGLSupport_vs2010_x64_release.lib")
#else
#pragma comment(lib, "BulletCollision_vs2010_x64_debug.lib")
#pragma comment(lib, "BulletDynamics_vs2010_x64_debug.lib")
// #pragma comment(lib, "BulletSoftBody_vs2010_x64_debug.lib")
// #pragma comment(lib, "ConvexDecomposition_vs2010_x64_debug.lib")
// #pragma comment(lib, "HACD_vs2010_x64_debug.lib")
#pragma comment(lib, "LinearMath_vs2010_x64_debug.lib")
#pragma comment(lib, "OpenGLSupport_vs2010_x64_debug.lib")
#endif

using namespace irr;

/********************  BEGIN OF THE GENERAL EVENT RECEIVER **********************/
class MyEventReceiver : public IEventReceiver
{
public:
	// We'll create a struct to record info on the mouse state
	struct SMouseState
	{
		core::position2di Position;
		bool LeftButtonDown;
		bool MiddleButtonDown;
		bool RightButtonDown;
		SMouseState() : LeftButtonDown(false) { }
	} MouseState;
	// This is the one method that we have to implement
	virtual bool OnEvent(const SEvent& event)
	{
		// Remember whether each key is down or up
		if(event.EventType == irr::EET_KEY_INPUT_EVENT)
		{
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
		}
		// Remember the mouse state
		if(event.EventType == irr::EET_MOUSE_INPUT_EVENT)
		{
			switch(event.MouseInput.Event)
			{
				case EMIE_LMOUSE_PRESSED_DOWN:
				{
					MouseState.LeftButtonDown = true;
					break;
				}
				case EMIE_LMOUSE_LEFT_UP:
				{
					MouseState.LeftButtonDown = false;
					break;
				}
				case EMIE_RMOUSE_PRESSED_DOWN:
				{
					MouseState.RightButtonDown = true;
					break;
				}
				case EMIE_RMOUSE_LEFT_UP:
				{
					MouseState.RightButtonDown = false;
					break;
				}
				case EMIE_MMOUSE_PRESSED_DOWN:
				{
					MouseState.MiddleButtonDown = true;
					break;
				}
				case EMIE_MMOUSE_LEFT_UP:
				{
					MouseState.MiddleButtonDown = false;
					break;
				}
				case EMIE_MOUSE_MOVED:
				{
					MouseState.Position.X = event.MouseInput.X;
					MouseState.Position.Y = event.MouseInput.Y;
					break;
				}
				default:
				{
					// We won't use the wheel
					break;
				}
			}
		}
		return false;
	}
	// This is used to check whether a key is being held down
	virtual bool IsKeyDown(EKEY_CODE keyCode) const
	{
		return KeyIsDown[keyCode];
	}
	MyEventReceiver()
	{
		for (u32 i=0; i<KEY_KEY_CODES_COUNT; ++i)
			KeyIsDown[i] = false;
	}
	const SMouseState & GetMouseState(void) const
	{
		return MouseState;
	}

private:
	// We use this array to store the current state of each key
	bool KeyIsDown[KEY_KEY_CODES_COUNT];
};

/********************  ~~END OF THE GENERAL EVENT RECEIVER **********************/


int main(void)
{
	/****************** screen dimensions: ***********************/
	u32 nWidth, nHeight;
	nWidth = (u32)GetSystemMetrics(SM_CXSCREEN);
	nHeight = (u32)GetSystemMetrics(SM_CYSCREEN);
	MyEventReceiver receiver;
	
	/****************** Irrlicht stuff: *********************/
	IrrlichtDevice* device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(nWidth, nHeight), 32u, true, true, false, &receiver);
	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();
	gui::IGUIEnvironment* env = device->getGUIEnvironment();
	// skybox and lights:
	smgr->setAmbientLight(video::SColorf(0.1f, 0.1f, 0.0f, 1.0f));
	scene::ILightSceneNode* nodSun = smgr->addLightSceneNode(
		(scene::ISceneNode*)0, core::vector3df(10000.f, 10000.f, 10000.f), video::SColorf(0.9f, 0.8f, 0.7f, 1.f), 18800.f, -1);
	// scene::ISceneNode* nodSkyDome = smgr->addSkyDomeSceneNode(driver->getTexture("c:/media/skydome12.jpg"));
	driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);
	scene::ISceneNode* skybox=smgr->addSkyBoxSceneNode(
		driver->getTexture("c:/media/irrlicht2_up.jpg"),
		driver->getTexture("c:/media/irrlicht2_dn.jpg"),
		driver->getTexture("c:/media/irrlicht2_lf.jpg"),
		driver->getTexture("c:/media/irrlicht2_rt.jpg"),
		driver->getTexture("c:/media/irrlicht2_ft.jpg"),
		driver->getTexture("c:/media/irrlicht2_bk.jpg"));
	driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);

	/****************** Bullet stuff: *********************/
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));
	
	// shapes:
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	   
	btCollisionShape* fallShape = new btSphereShape(1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);

	btDefaultMotionState* fallMotionState =
		new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	fallShape->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
	dynamicsWorld->addRigidBody(fallRigidBody);

	/****************************** Irrlicht nodes: **************************************/
	// the table
	scene::IMesh* mshTable = smgr->addHillPlaneMesh("base", core::dimension2d<f32>(32.f, 32.f), core::dimension2d<u32>(4, 4));
	scene::ISceneNode* nodTable = smgr->addMeshSceneNode(mshTable);
	nodTable->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	nodTable->setMaterialFlag(video::EMF_LIGHTING, true);
	nodTable->setMaterialTexture(0, driver->getTexture("c:/media/~map/Brick_C.jpg"));

	// the ball
	// scene::IAnimatedMesh* mshBall = smgr->addSphereMesh("Ball001", 1.f);
	scene::IAnimatedMesh* mshBall = smgr->getMesh("c:/media/sphere000.obj");
	scene::IAnimatedMeshSceneNode* nodBall = smgr->addAnimatedMeshSceneNode(mshBall);	// with mesh for shadow volume scene node
	// scene::ISceneNode* nodBall = smgr->addSphereSceneNode(1);	// if you enable this, there will be no shadow anymore
	nodBall->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	nodBall->setMaterialFlag(video::EMF_LIGHTING, true);
	nodBall->setMaterialTexture(0, driver->getTexture("c:/media/wall.jpg"));
	nodBall->addShadowVolumeSceneNode(mshBall);

	btTransform trans;		// just not to see the ball direct at the zero point at the start
	fallRigidBody->getMotionState()->getWorldTransform(trans);
	nodBall->setPosition(core::vector3df(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
	nodBall->setRotation(core::vector3df(trans.getRotation().getX(), trans.getRotation().getY(),trans.getRotation().getZ()));

	/**************************** Irrlicth Camera *********************************/
	scene::ICameraSceneNode* camMain = smgr->addCameraSceneNodeFPS(0, 20, 0.01f, -1, (irr::SKeyMap*)0, 0, false);
	camMain->setPosition(core::vector3df(0.f, 4.f, 30.f));
	camMain->setTarget(core::vector3df(0.f, 4.f, 0.f));

	bool bForce = false;
	device->getCursorControl()->setVisible(false);
	/***************************** now the main loop: ***********************************/
	while(device->run())
	{
		driver->beginScene(true, true, video::SColor(255, 32, 32, 192));
		smgr->drawAll();
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
		
		if(receiver.IsKeyDown(irr::KEY_KEY_B))
		{
			// fallRigidBody->applyCentralImpulse(btVector3(1., 0., 0.));
			// fallRigidBody->applyCentralForce(btVector3(0., 12., 0.));
			// bForce = true;
			fallRigidBody->applyCentralImpulse(btVector3(0.1, 0.1, 0.));
		}
		else
		{
			fallRigidBody->applyCentralImpulse(btVector3(0., 0., 0.));
		}
		
		btTransform trans;
		fallRigidBody->getMotionState()->getWorldTransform(trans);

		// std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
		/*****************************  Irrlicth stuff: ******************************************/
		nodBall->setPosition(core::vector3df(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
		nodBall->setRotation(core::vector3df(trans.getRotation().getX(), trans.getRotation().getY(),trans.getRotation().getZ()));

		driver->endScene();
	}

	/******************* Release Bullet resources: *******************/
	dynamicsWorld->removeRigidBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;

	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	delete fallShape;

	delete groundShape;

	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	/**************** release Irrlicht resources: ****************/
	device->getCursorControl()->setVisible(true);
	device->drop();
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// tests materials, tests also Bullet physics...

// BUGS: See below link errors: 

#include <iostream>
#include <windows.h>
#include "irrlicht.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>


#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")

#pragma comment(lib, "BulletCollision_vs2010_x64_release.lib")					// BulletCollision_vs2010_x64_release.lib
#pragma comment(lib, "BulletDynamics_vs2010_x64_release.lib")					// BulletDynamics_vs2010_x64_release.lib
#pragma comment(lib, "BulletSoftBody_vs2010_x64_release.lib")					// BulletSoftBody_vs2010_x64_release.lib
#pragma comment(lib, "ConvexDecomposition_vs2010_x64_release.lib")				// ConvexDecomposition_vs2010_x64_release.lib
#pragma comment(lib, "HACD_vs2010_x64_release.lib")								// HACD_vs2010_x64_release.lib
#pragma comment(lib, "LinearMath_vs2010_x64_release.lib")						// LinearMath_vs2010_x64_release.lib
#pragma comment(lib, "OpenGLSupport_vs2010_x64_release.lib")					// OpenGLSupport_vs2010_x64_release.lib

// #pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

#define PI 3.14159f

#define FREE_LIFT		true
#define DELTA_THETA		0.0001f

using namespace irr;

// Passes bullet's orientation to irrlicht
void UpdateRender(btRigidBody *TObject);
// 
void CreateStartScene(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects);
// Create a box rigid body
void CreateBox(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, const btVector3 &TPosition, const core::vector3df &TScale, btScalar TMass);
// Create a sphere
void CreateSphere(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, const btVector3 &TPosition, btScalar TRadius, btScalar TMass);
// ...
void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler);
// ...
void UpdatePhysics(btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, u32 TDeltaTime);
// Removes all objects from the world
void ClearObjects(btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects);
// something nice:
int GetRandInt(int TMax);


/********************  BEGIN OF THE GENERAL EVENT RECEIVER **********************/
class MyEventReceiver : public IEventReceiver
{
public:
	// We'll create a struct to record info on the mouse state
	struct SMouseState
	{
		core::position2di Position;
		bool LeftButtonDown;
		bool MiddleButtonDown;
		bool RightButtonDown;
		SMouseState() : LeftButtonDown(false) { }
	} MouseState;
	// This is the one method that we have to implement
	virtual bool OnEvent(const SEvent& event)
	{
		// Remember whether each key is down or up
		if(event.EventType == irr::EET_KEY_INPUT_EVENT)
		{
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
		}
		// Remember the mouse state
		if(event.EventType == irr::EET_MOUSE_INPUT_EVENT)
		{
			switch(event.MouseInput.Event)
			{
				case EMIE_LMOUSE_PRESSED_DOWN:
				{
					MouseState.LeftButtonDown = true;
					break;
				}
				case EMIE_LMOUSE_LEFT_UP:
				{
					MouseState.LeftButtonDown = false;
					break;
				}
				case EMIE_RMOUSE_PRESSED_DOWN:
				{
					MouseState.RightButtonDown = true;
					break;
				}
				case EMIE_RMOUSE_LEFT_UP:
				{
					MouseState.RightButtonDown = false;
					break;
				}
				case EMIE_MMOUSE_PRESSED_DOWN:
				{
					MouseState.MiddleButtonDown = true;
					break;
				}
				case EMIE_MMOUSE_LEFT_UP:
				{
					MouseState.MiddleButtonDown = false;
					break;
				}
				case EMIE_MOUSE_MOVED:
				{
					MouseState.Position.X = event.MouseInput.X;
					MouseState.Position.Y = event.MouseInput.Y;
					break;
				}
				default:
				{
					// We won't use the wheel
					break;
				}
			}
		}
		return false;
	}
	// This is used to check whether a key is being held down
	virtual bool IsKeyDown(EKEY_CODE keyCode) const
	{
		return KeyIsDown[keyCode];
	}
	MyEventReceiver()
	{
		for (u32 i=0; i<KEY_KEY_CODES_COUNT; ++i)
			KeyIsDown[i] = false;
	}
	const SMouseState & GetMouseState(void) const
	{
		return MouseState;
	}

private:
	// We use this array to store the current state of each key
	bool KeyIsDown[KEY_KEY_CODES_COUNT];
};

/********************  ~~END OF THE GENERAL EVENT RECEIVER **********************/

// main function:
int main()
{
	IrrlichtDevice* device = createDevice(video::EDT_DIRECT3D9, core::dimension2d<u32>(1600, 900));
	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();
	device->setWindowCaption(L"Test program for Irrlicht");
	scene::ISceneCollisionManager* colmgr = smgr->getSceneCollisionManager();
	// Receiver:
	MyEventReceiver receiver;
	device->setEventReceiver(&receiver);

	// now bullet variables:
	btDiscreteDynamicsWorld *World;
	core::list<btRigidBody*> Objects;

	// now initialize Bullet:
	btDefaultCollisionConfiguration*		CollisionConfiguration = new btDefaultCollisionConfiguration();
	btBroadphaseInterface*					BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	btCollisionDispatcher*					Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	btSequentialImpulseConstraintSolver*	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0, -10, 0));	// added

	smgr->setAmbientLight(video::SColorf(0.1f, 0.1f, 0.0f, 1.0f));
	scene::ILightSceneNode* nodSun = smgr->addLightSceneNode(
		(scene::ISceneNode*)0, core::vector3df(-10000.f, 1000.f, -10000.f), video::SColorf(0.9f, 0.8f, 0.7f, 1.f), 18000.f, -1);
	scene::ISceneNode* nodSkyDome = smgr->addSkyDomeSceneNode(driver->getTexture("c:/media/skydome12.jpg"));

	// Collision detection variables: meta-triangle selector and triangle selector self:
	scene::IMetaTriangleSelector* met = smgr->createMetaTriangleSelector();	// meta-triangle selector
	scene::ITriangleSelector* tri = 0;// triangle selector: the same will be declared, 
	//(re)created for every node and added to the meta-triangle-selector at its time
	
	/***************** BEGIN OF HEIGHTMAP TERRAIN *******************/
	scene::ITerrainSceneNode* ter = smgr->addTerrainSceneNode(
		"c:/media/terrain-heightmap.bmp",			// heightmap
		0,											// parent node
		-1,											// node id
		core::vector3df(-2000.f, -4200.f, -2000.f),	// position
		core::vector3df(0.f, 0.f, 0.f),				// rotation
		core::vector3df(2000.f, 44.4f, 2000.f),		// scale
		video::SColor ( 255, 255, 255, 255 ),		// vertexColor
		5,											// maxLOD
		scene::ETPS_17,								// patchSize
		4											// smoothFactor
		);

	ter->setMaterialFlag(video::EMF_LIGHTING, false);
	ter->setMaterialTexture(0, driver->getTexture("c:/media/terrain-texture.jpg"));
	// ter->setMaterialTexture(1, driver->getTexture("c:/media/detailmap3.jpg"));
	ter->setMaterialType(video::EMT_DETAIL_MAP);
	ter->scaleTexture(1.0f, 20.0f);
	/***************** ~~END OF HEIGHTMAP TERRAIN *******************/

	// a sphere for Bullet Physics:
	
	/***************** BEGIN OF TEST A HALLWAY **************/
	core::array<scene::IAnimatedMesh*> mshsHallwayTest;
	mshsHallwayTest.reallocate(2);
	mshsHallwayTest[0] = smgr->getMesh("c:/media/HallwayTudor2/stone001_001.obj");
	mshsHallwayTest[1] = smgr->getMesh("c:/media/HallwayTudor2/traverse_001.000.obj");
	// printf("allocated size: %d\n", mshsHallwayTest.allocated_size());
	core::array<scene::IAnimatedMeshSceneNode*> nodsHallwayTest;
	nodsHallwayTest.reallocate(2);
	for(int i=0; i<mshsHallwayTest.allocated_size(); i++)
	{
		nodsHallwayTest[i] = smgr->addAnimatedMeshSceneNode(mshsHallwayTest[i]);
	}
	for(int i=0; i<mshsHallwayTest.allocated_size(); i++)
	{
		if(nodsHallwayTest[i])
		{
			nodsHallwayTest[i]->setScale(core::vector3df(8.f, 8.f, 8.f));
			nodsHallwayTest[i]->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			nodsHallwayTest[i]->setMaterialFlag(video::EMF_LIGHTING, true);
			nodsHallwayTest[i]->getMaterial(0).setTexture(0, driver->getTexture("C:/media/~map/stone_texture_7___seamless_by_agf81-d3eih8f.jpg"));
			// nodHallwayTest->getMaterial(0).getTextureMatrix(0).setTextureScale(512, 512);
			if(met)
			{
				tri = smgr->createTriangleSelector(mshsHallwayTest[i], nodsHallwayTest[i]);
				if(!nodsHallwayTest[i]->getTriangleSelector()) nodsHallwayTest[i]->setTriangleSelector(tri);
				met->addTriangleSelector(tri);
				// tri->drop;		// not yet!
			}
		}
	}
	nodsHallwayTest[1]->setPosition(nodsHallwayTest[1]->getPosition()+core::vector3df(0.f, 2.f, 0.f));
	/***************** ~~END OF TEST A HALLWAY **************/

	/***************** BEGIN OF THE MOBILE ELEMENT #1 **************/
	scene::IAnimatedMesh* mshMobEl = smgr->getMesh("c:/media/HallwayTudor2/cylinder001_001.obj");
	scene::IAnimatedMeshSceneNode* nodMobEl = smgr->addAnimatedMeshSceneNode(mshMobEl);;
	if(nodMobEl && nodsHallwayTest[0])
	{
		nodMobEl->setScale(nodsHallwayTest[0]->getScale());
		nodMobEl->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		nodMobEl->setMaterialFlag(video::EMF_LIGHTING, true);
		nodMobEl->getMaterial(0).setTexture(0, driver->getTexture("c:/media/~map/stone_blocks_by_agf81.jpg"));
		/*********************	**********************/
		if(met)
		{
			tri = smgr->createTriangleSelector(mshMobEl, nodMobEl);
			if(!nodMobEl->getTriangleSelector()) nodMobEl->setTriangleSelector(tri);
			met->addTriangleSelector(tri);
			// tri->drop();		// not yet!
		}
	}

	scene::IMetaTriangleSelector* metMobEl = smgr->createMetaTriangleSelector();
	
	f32 fElVPos = 0.f;		// this next variables will be used to demo-animate the element
	f32 fElHPos = 0.f;
	f32 fth = -PI/2.f;
	bool bIsInLift = false;
	/***************** ~~END OF THE MOBILE ELEMENT #1 **************/

	/**************** BEGIN OF VARIOUS TEST NODES ********************/
	core::array<scene::IAnimatedMesh*> mshsTest;
	mshsTest.push_back(smgr->getMesh("c:/media/HallwayTudor2/LiftingElement001.obj"));
	mshsTest.push_back(smgr->getMesh("c:/media/HallwayTudor2/Torus001.obj"));
	mshsTest.push_back(smgr->getMesh("c:/media/HallwayTudor2/Cone001.obj"));
	mshsTest.push_back(smgr->getMesh("c:/media/HallwayTudor2/Ball001.obj"));
	mshsTest.push_back(smgr->getMesh("c:/media/HallwayTudor2/Fifi001.obj"));
	core::array<video::ITexture*> texsTest;
	texsTest.push_back(driver->getTexture("c:/media/~map/Roofing_Shingles_Wood.jpg"));
	texsTest.push_back(driver->getTexture("c:/media/~map/seamless_bolted_texture_by_fantasystock.jpg"));
	texsTest.push_back(driver->getTexture("c:/media/~map/seamless_metal_plate_by_hhh316-d4iq7o9.jpg"));
	texsTest.push_back(driver->getTexture("c:/media/~map/stone_blocks_by_agf81.jpg"));
	texsTest.push_back(driver->getTexture("c:/media/~map/seamless_metal_plate_by_hhh316-d4ink0a.jpg"));
	u32 j = 0;			// counts 

	core::array<scene::IAnimatedMeshSceneNode*> nodsTest;
	for(u32 i=0; i<5; i++)
	{
		if(mshsTest[i])
		{
			nodsTest.push_back(smgr->addAnimatedMeshSceneNode(mshsTest[i],
				(scene::ISceneNode*)0, -1, 
				core::vector3df(i*5.f-10.f, i*20.f, -5+i*0.9f) - core::vector3df(40.f, 10.f, 40.f),
				core::vector3df(0.f, 0.f, 0.f),
				core::vector3df(1.f, 1.f, 1.f)*(1.f+i*0.25))
				);
		}
		printf("here\n");
		if(nodsTest[i])
		{
			{
				nodsTest[i]->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
				nodsTest[i]->setMaterialFlag(video::EMF_LIGHTING, true);
				if(texsTest[j]) nodsTest[i]->getMaterial(0).setTexture(0, texsTest[j]);
				j+=(i<texsTest.allocated_size());
			}
		}

	}

	/**************** ~~END OF VARIOUS TEST NODES ********************/

	/******************************* BEGIN OF SHIP NODE(S) ******************************/
	// the air ship will become a concave bullet body and the camera will stay in 
	scene::IAnimatedMesh* mshAirShip = smgr->getMesh("c:/media/HallwayTudor2/AirShip.000.obj");
	scene::IAnimatedMeshSceneNode* nodAirShip = smgr->addAnimatedMeshSceneNode(mshAirShip, (scene::ISceneNode*)0, -1,
		core::vector3df(120.f, 0.f, 0.f),
		core::vector3df(0.f, 0.f, 0.f),
		core::vector3df(1.f, 1.f, 1.f));
	nodAirShip->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	nodAirShip->setMaterialFlag(video::EMF_LIGHTING, true);
	nodAirShip->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	
	if(met)
	{
		tri = smgr->createTriangleSelector(mshAirShip, nodAirShip);
		if(!nodAirShip->getTriangleSelector()) nodAirShip->setTriangleSelector(tri);
		met->addTriangleSelector(tri);
		// tri->drop();		// ...also it can be
	}
	scene::IMetaTriangleSelector* metAirShip = smgr->createMetaTriangleSelector();

	/******************************* ~~END OF SHIP NODE(S) ******************************/

	/*************************************** BEGIN OF TEST BUTTONS'S TEXTURE ********************************************
	scene::IAnimatedMesh* mshTest = smgr->getMesh("C:/media/Building N090914/cubCmdB003.obj");
	core::array<scene::ISceneNode*> nodsTest;
	for(u32 i=0; i<16; i++)
	{
		nodsTest.push_back(smgr->addAnimatedMeshSceneNode(mshTest));
	}
	for(u32 i=0; i<16; i++)
	{
		if(nodsTest[i])
		{
			nodsTest[i]->setPosition(core::vector3df(12.f+(f32)(i%4), (f32)((u32)(i/4)), 24.f));
			nodsTest[i]->setMaterialFlag(video::EMF_LIGHTING, true);
			nodsTest[i]->getMaterial(1).setTexture(0, driver->getTexture("c:/media/stones.jpg"));
		}

	}
	*************************************** ~~END OF TEST BUTTONS'S TEXTURE ********************************************/
	// no nodes anymore, we can drop the triangle selector....
	tri->drop();			// never forget again!!!						NEVER!!!!!!!

	/************************************** BEGIN OF MAIN CAMERA *******************************************************/
	SKeyMap keyMap[] = 
	{
		{EKA_MOVE_FORWARD,		KEY_UP		},	{EKA_MOVE_FORWARD,		KEY_KEY_W	},
		{EKA_MOVE_BACKWARD,		KEY_DOWN	},	{EKA_MOVE_BACKWARD,		KEY_KEY_S	},
		{EKA_STRAFE_LEFT,		KEY_LEFT	},	{EKA_STRAFE_LEFT,		KEY_KEY_A	},
		{EKA_STRAFE_RIGHT,		KEY_RIGHT	},	{EKA_STRAFE_RIGHT,		KEY_KEY_D	},
		{EKA_JUMP_UP,			KEY_SPACE	},	{EKA_JUMP_UP,			KEY_SPACE	},
	};
	// now the FPS camera self:
	scene::ICameraSceneNode* camMain = smgr->addCameraSceneNodeFPS(
		(scene::ISceneNode*)0, 50.f, 0.012f, -1, keyMap, sizeof(keyMap), false, 0.24f
		);
	camMain->setFarValue(9000.1f);
	camMain->setNearValue(0.1f);
	camMain->setPosition(core::vector3df(0.f, 1.7f, 0.f));
	// collision response animator for the main camera:
	scene::ISceneNodeAnimatorCollisionResponse* cra = smgr->createCollisionResponseAnimator(met, camMain, 
		core::vector3df(1.f, 3.5f, 1.f), core::vector3df(0.f, -.98f, 0.f), core::vector3df(0.f, 0.5f, 0.f)
		);
	camMain->addAnimator(cra);
	cra->drop();
	// for collide the camera with a moving node (an elevator, a flying ship) and not to go through:

	// next two variables were for manual created collision response with moving objects: TO DO: try to remove the need for them in the program!
	core::vector3df posEle0 = nodMobEl->getPosition();
	core::vector3df posEle1 = nodMobEl->getPosition();

	/************************************** ~~END OF MAIN CAMERA *******************************************************/

	//
	core::vector3df posAirShip0 = nodAirShip->getPosition();
	device->getCursorControl()->setVisible(false);	// 

	u32 TimeStamp = device->getTimer()->getTime();
	u32 DeltaTime = 0;
	/************************************** BEGIN OF MAIN LOOP *******************************************************/
	while(device->run() && driver)
	{
		// 
		DeltaTime = device->getTimer()->getTime() - TimeStamp;
		TimeStamp = device->getTimer()->getTime();
		UpdatePhysics(World, Objects, DeltaTime);
		driver->beginScene(true, true, video::SColor(255, 0, 0, 255));
		// 
		if(receiver.IsKeyDown(irr::KEY_KEY_X))
		{
			CreateStartScene(device, World, Objects);
			Sleep(200);
		}
		if(receiver.IsKeyDown(irr::KEY_KEY_1))
		{
			CreateBox(device, World, Objects, btVector3(GetRandInt(240) - 5.0f, 840.0f, GetRandInt(240) - 25.0f), core::vector3df(GetRandInt(20) + 0.1f, GetRandInt(10) + 0.1f, GetRandInt(10) + 0.1f), 1.0f);
			Sleep(200);
		}
		if(receiver.IsKeyDown(irr::KEY_KEY_2))
		{
			CreateSphere(device, World, Objects, btVector3(GetRandInt(40) - 5.0f, 727.0f, GetRandInt(40) - 5.0f), GetRandInt(4)/1.0f + 0.1f, 1.0f);
			Sleep(200);
		}

		/**  Animation of the mobile element #1 **/
		if(bIsInLift||FREE_LIFT)
		{
			fth+=DELTA_THETA;
			if(fth>360.f) fth=0.f;
			fElVPos = 100.f*(1.f+sinf(fth));
			fElHPos = 100.f*(cosf(fth));
		}
		// nodMobEl->setPosition(core::vector3df(0.f, fElVPos, 0.f));
		// nodMobEl->updateAbsolutePosition();

		/********************************		********************************/
		core::vector3df posEllipsoidCam;
		core::vector3df traEllipsoidCam;
		core::vector3df dirEllipsoidCam;
		core::triangle3df triContact;
		core::vector3df posHitContact;
		bool bIsFalling;
		scene::ISceneNode* nodContact;
		core::vector3df posContact = colmgr->getCollisionResultPosition(met, posEllipsoidCam, traEllipsoidCam, dirEllipsoidCam, 
			triContact, posHitContact, bIsFalling, nodContact);
		/********************************		
		
		core::aabbox3d<f32> boxMobEl = nodMobEl->getTransformedBoundingBox();
		bIsInLift = boxMobEl.isPointInside(camMain->getPosition());
		if(bIsInLift)
		{
			posEle1 = nodMobEl->getPosition();
			posEle1+=core::vector3df(0.0f, 0.05f, 0.0f);
			posEle0 = nodMobEl->getPosition();
			// REVISION REQUIRED
			if(!FREE_LIFT)
			{
				nodMobEl->setPosition(core::vector3df(0.f, fElVPos, 0.f));	// move the mobile element
			}
			// camMain->setPosition(camMain->getPosition()+posEle1-posEle0);		// camera following the moving object done another way...
			// cra->setGravity(core::vector3df(0.f, 0.f, 0.f));		// do not use it!!!
		}
		if(FREE_LIFT)
		{
			nodMobEl->setPosition(core::vector3df(0.f, fElVPos, 0.f));			// move the mobile element
			nodAirShip->setPosition(posAirShip0 + 2.f*core::vector3df(0.f, fElVPos, 0.f));
		}

		********************************/
		// draw all
		smgr->drawAll();
		// end scene
		driver->endScene();
	}
	/************************************** ~~END OF MAIN LOOP *******************************************************/

	mshsHallwayTest.clear();
	nodsHallwayTest.clear();

	mshsTest.clear();
	texsTest.clear();
	nodsTest.clear();
	
	// delete Bullet objects used:
	ClearObjects(World, Objects);
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;
	delete World;

	// delete Irrlicht objects used:
	metMobEl->drop();
	metAirShip->drop();
	met->drop();
	device->drop();
	return 0;
}


// Creates a base box
void CreateStartScene(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects)
{
	ClearObjects(World, Objects);
	CreateBox(device, World, Objects, btVector3(0.0f, 0.0f, 0.0f), core::vector3df(10.0f, 0.5f, 10.0f), 0.0f);
}

// Create a box rigid body
void CreateBox(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, const btVector3 &TPosition, const core::vector3df &TScale, btScalar TMass) 
{
	video::IVideoDriver* irrDriver = device->getVideoDriver();
	scene::ISceneManager* irrScene = device->getSceneManager();

	// Create an Irrlicht cube
	scene::ISceneNode *Node = irrScene->addCubeSceneNode(1.0f);
	Node->setScale(TScale);
	Node->setMaterialFlag(video::EMF_LIGHTING, 1);
	Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, irrDriver->getTexture("c:/media/~map/Copper01.jpg"));

	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(TPosition);

	// Give it a default MotionState
	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(TScale.X * 0.5f, TScale.Y * 0.5f, TScale.Z * 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);

	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(TMass, LocalInertia);

	// Create the rigid body object
	btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer((void *)(Node));

	// Add it to the world
	World->addRigidBody(RigidBody);
	Objects.push_back(RigidBody);
}


// Create a sphere rigid body
void CreateSphere(IrrlichtDevice* device, btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, const btVector3 &TPosition, btScalar TRadius, btScalar TMass) 
{
	video::IVideoDriver* irrDriver = device->getVideoDriver();
	scene::ISceneManager* irrScene = device->getSceneManager();

	// Create an Irrlicht sphere
	scene::ISceneNode *Node = irrScene->addSphereSceneNode(TRadius, 32);
	Node->setMaterialFlag(video::EMF_LIGHTING, 1);
	Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, irrDriver->getTexture("c:/media/~map/Concrete_Salt.jpg"));

	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(TPosition);

	// Give it a default MotionState
	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btCollisionShape *Shape = new btSphereShape(TRadius);

	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(TMass, LocalInertia);

	// Create the rigid body object
	btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer((void *)(Node));

	// Add it to the world
	World->addRigidBody(RigidBody);
	Objects.push_back(RigidBody);
}


// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void UpdatePhysics(btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects, u32 TDeltaTime)
{

	World->stepSimulation(TDeltaTime * 0.001f, 60);

	btRigidBody *TObject;
	// Relay the object's orientation to irrlicht
	for(core::list<btRigidBody *>::Iterator it = Objects.begin(); it != Objects.end(); ++it) {

		// UpdateRender(*it);	// modified by Claudiu Ciutacu
		scene::ISceneNode *Node = static_cast<scene::ISceneNode *>((*it)->getUserPointer());
		TObject = *it;

		// Set position
		btVector3 Point = TObject->getCenterOfMassPosition();
		Node->setPosition(core::vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

		// Set rotation
		btVector3 EulerRotation;
		QuaternionToEuler(TObject->getOrientation(), EulerRotation);
		Node->setRotation(core::vector3df(EulerRotation[0], EulerRotation[1], EulerRotation[2]));

	}
}

// Converts a quaternion to an euler angle
void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler) 
{
	btScalar W = TQuat.getW();
	btScalar X = TQuat.getX();
	btScalar Y = TQuat.getY();
	btScalar Z = TQuat.getZ();
	float WSquared = W * W;
	float XSquared = X * X;
	float YSquared = Y * Y;
	float ZSquared = Z * Z;

	TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
	TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	TEuler *= core::RADTODEG;
}

// Passes bullet's orientation to irrlicht
void UpdateRender(btRigidBody *TObject)
{
	scene::ISceneNode* Node = static_cast<scene::ISceneNode *>(TObject->getUserPointer());

	// Set position
	btVector3 Point = TObject->getCenterOfMassPosition();
	Node->setPosition(core::vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

	// Set rotation
	core::vector3df Euler;
	const btQuaternion& TQuat = TObject->getOrientation();
	core::quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
	q.toEuler(Euler);
	Euler *= core::RADTODEG;
	Node->setRotation(Euler);
}

// Removes all objects from the world
void ClearObjects(btDiscreteDynamicsWorld *World, core::list<btRigidBody *> Objects) 
{

	for(core::list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
		btRigidBody *Object = *Iterator;

		// Delete irrlicht node
		scene::ISceneNode *Node = static_cast<scene::ISceneNode *>(Object->getUserPointer());
		Node->remove();

		// Remove the object from the world
		World->removeRigidBody(Object);

		// Free memory
		delete Object->getMotionState();
		delete Object->getCollisionShape();
		delete Object;
	}

	Objects.clear();
}

// something nice:
int GetRandInt(int TMax) 
{ 
	return rand() % TMax; 
}


/*********************************************************************************************************************

Error	LNK2016	Absolute symbol '@comp.id' used as target of REL32 relocation	IrrlichtTestMaterials	D:\PROJECTS.TEST.016\IrrlichtTestMaterials\test.010.a01\IrrlichtTestMaterials\main.obj	1
Solved: rebuild all and then will get!!!!
href	"http://www.pcreview.co.uk/threads/error-lnk2016-absolute-symbol-comp-id-used-as-target-of-rel32relocation.3625757/"

Error	LNK2038	mismatch detected for 'RuntimeLibrary': value 'MT_StaticRelease' doesn't match value 'MD_DynamicRelease' in main.obj	IrrlichtTestMaterials	D:\PROJECTS.TEST.016\IrrlichtTestMaterials\test.010.a01\IrrlichtTestMaterials\BulletCollision_vs2010_x64_release.lib(btActivatingCollisionAlgorithm.obj)	1
Solved: Configuration Properties -> C/C++ -> Code Generation -> Runtime Library  = Multi-threaded (/MT)
href	"http://stackoverflow.com/questions/13444816/bullet-physics-linking-error-in-visual-studio"


see also:
href	"http://irrlicht3d.org/wiki/index.php?n=Main.Tutorials"

*********************************************************************************************************************/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// http://irrlicht.sourceforge.net/forum/viewtopic.php?p=223918#p223918


#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include <iostream>

#pragma comment(lib, "irrlicht.lib")

#define CX_COMPILE_VS_DEBUG_MODE

#if !defined(CX_COMPILE_VS_DEBUG_MODE)
#pragma comment(lib, "BulletCollision_vs2010_x64_release.lib")
#pragma comment(lib, "BulletDynamics_vs2010_x64_release.lib")
#pragma comment(lib, "BulletSoftBody_vs2010_x64_release.lib")
#pragma comment(lib, "ConvexDecomposition_vs2010_x64_release.lib")
#pragma comment(lib, "HACD_vs2010_x64_release.lib")
#pragma comment(lib, "LinearMath_vs2010_x64_release.lib")
#pragma comment(lib, "OpenGLSupport_vs2010_x64_release.lib")
#else
#pragma comment(lib, "BulletCollision_vs2010_x64_debug.lib")
#pragma comment(lib, "BulletDynamics_vs2010_x64_debug.lib")
#pragma comment(lib, "BulletSoftBody_vs2010_x64_debug.lib")
#pragma comment(lib, "ConvexDecomposition_vs2010_x64_debug.lib")
#pragma comment(lib, "HACD_vs2010_x64_debug.lib")
#pragma comment(lib, "LinearMath_vs2010_x64_debug.lib")
#pragma comment(lib, "OpenGLSupport_vs2010_x64_debug.lib")
#endif


using namespace irr;


using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler)
{
	btScalar W = TQuat.getW();
	btScalar X = TQuat.getX();
	btScalar Y = TQuat.getY();
	btScalar Z = TQuat.getZ();
	float WSquared = W * W;
	float XSquared = X * X;
	float YSquared = Y * Y;
	float ZSquared = Z * Z;

	TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
	TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	TEuler *= core::RADTODEG;
}

int main(int argc, char** argv)
{
	//Creating the irrlicht device
	IrrlichtDevice *device =createDevice(EDT_OPENGL, dimension2d<u32>(640, 480), 16, false, false, false, 0);

	//setting pointers to the device!
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();
	IGUIEnvironment* guienv = device->getGUIEnvironment();

	//creating the 3d ball
	ISceneNode *irrball = smgr->addSphereSceneNode();
	irrball->setMaterialTexture( 0, driver->getTexture("../../media/wall.jpg") );
	irrball->setMaterialFlag(EMF_LIGHTING,false);

	//creating the camera
	ICameraSceneNode *irrcam = smgr->addCameraSceneNode();
	irrcam->setPosition(vector3df(0,0,-70));

	//creating the broadphase
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();

	//creating the physics properties configuration
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//creating the solvers
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	//Making the dynamic world
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

	//setting the gravity
	dynamicsWorld->setGravity(btVector3(0,-10,0));


	//Its time to create the shapes for our collision
	//Creating the ground( a simple plane )
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

	//Creating the ball shape
	btCollisionShape* fallShape = new btSphereShape(1);

	//its time to create the ground object
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));

	//making the ground rigidody
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

	//adding the ground to the physics world
	dynamicsWorld->addRigidBody(groundRigidBody);

	//its time to create the ball object!!!
	btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)));

	//adding and calculating the ball sphere mas
	btScalar mass = 1;
	btVector3 fallInertia(0,0,0);
	fallShape->calculateLocalInertia(mass,fallInertia);

	//constructing the rigidody(adding info,etc)
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);

	//Creating the ball rigidbody
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);

	//And offcourse add it to the world
	dynamicsWorld->addRigidBody(fallRigidBody);

	btTransform trans;
	btVector3 rot;

	while (device->run())
	{
		//getting the world positions(x,y,z)
		fallRigidBody->getMotionState()->getWorldTransform(trans);

		//converting the bullet rotation axes so we can use it with our irrlicht node
		QuaternionToEuler(trans.getRotation(),rot);

		//adding the position and rotation to the node
		irrball->setPosition(vector3df(trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ()));
		irrball->setRotation(vector3df(rot.getX(),rot.getY(),rot.getZ()));
		//stepping the simulation
		dynamicsWorld->stepSimulation(1/60.f,10);

		std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

		driver->beginScene(true, true, SColor(0,200,200,200));

		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();
	}

	// Clean up behind ourselves like good little programmers
	dynamicsWorld->removeRigidBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;

	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;


	delete fallShape;

	delete groundShape;


	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	device->drop();

	return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
