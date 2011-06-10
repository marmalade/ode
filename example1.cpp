//  hello.cpp by  Kosei Demura

// TIM TODO: IMPLEMENT EXTERNAL REFS ON TOP OF S3E / IWGX
 
#include <ode/ode.h>
#include "IwGx.h"
#include "s3eDevice.h"
#include "s3eKeyboard.h"
 
#ifdef  dDOUBLE
#define dsDrawSphere  dsDrawSphereD
#endif

typedef float dReal;
//typedef float dMass;
 
//static dWorldID world;   // For kinetics calculation
//dBodyID ball;   // Ball
//const dReal radius = 0.2, mass = 1.0;  // Radius of ball [m], weight of ball [kg]

static dWorldID world; // World for kinetics calculation
static dSpaceID space; // Space for collision detection
static dGeomID ground;
static dJointGroupID contactgroup; // As for contact group details ODE manual
const dReal radius = 0.2f, mass = 1.0f; // Radius [ m ] and mass [ m ]
 
// The last time was just body for kinetics calculation, but this time it meaning that geom for collision detection joins it defined ball object with structure.
typedef struct
{
	dBodyID body; // For kinetics calculation
	dGeomID geom; // For collision detection
} MyObject;
MyObject ball; // Ball object
 
// Call-back function of collision detection
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 4; // As for the upper limit of contact score without forgetting 4 static, attaching.
	
	dContact contact [ N ];
	 
	int isGround = ((ground == o1) || (ground == o2));
	int n = dCollide (o1, o2, N, &contact [ 0 ].geom, sizeof (dContact)); // As for n collision score
	if (isGround)
	{ 
		// The flag of the ground stands, collision detection function can be used
		for (int i = 0; i < n; i++)
		{
			contact[i].surface.mode = dContactBounce; // Setting the coefficient of rebound of the land
			contact[i].surface.bounce = 0.75; // (0.0 - 1.0) as for coefficient of rebound from 0 up to 1
			contact[i].surface.bounce_vel = 0.0; // (0.0 or more) the lowest speed which is necessary for rally
			 
			// Contact joint formation
			dJointID c = dJointCreateContact (world, contactgroup, &contact [ i ]);
			
			// Restraining two geometry which contact with the contact joint
			dJointAttach (c, dGeomGetBody (contact [ i ] .geom.g1),
			dGeomGetBody (contact [ i ] .geom.g2));
		}
	}
}

static void simLoop (int pause);
 
//------------------------------------------------------------------------------
static float s_Scale = 0x1000;	// Convert from float scale of example to fixed scale for IwGx
//------------------------------------------------------------------------------
void dsDrawSphere(const dReal* pos, const dReal* R, const dReal radius)
{
	CIwSphere sph;
	sph.t.x = (int32)(pos[0] * s_Scale);
	sph.t.y = (int32)(pos[1] * s_Scale);
	sph.t.z = (int32)(pos[2] * s_Scale);
	sph.SetRadius((int32)(radius * s_Scale));
	IwGxDebugPrimSphere(sph);
}
//------------------------------------------------------------------------------
void dsSetViewpoint(const dReal* pos, const dReal* eulers)
{
	CIwMat view;
	
	CIwMat rx, ry, rz;
	rx.SetRotX(IW_ANGLE_FROM_DEGREES(eulers[0]));
	ry.SetRotY(IW_ANGLE_FROM_DEGREES(eulers[1]));
	rz.SetRotZ(IW_ANGLE_FROM_DEGREES(eulers[2]));
	view = rx * ry * rz;

	view.t.x = (int32)(pos[0] * s_Scale);
	view.t.y = (int32)(pos[1] * s_Scale);
	view.t.z = (int32)(pos[2] * s_Scale);

	IwGxSetViewMatrix(&view);
}
//------------------------------------------------------------------------------
void dsSimulationLoop()
{
	while (!s3eDeviceCheckQuitRequest() && !s3eKeyboardAnyKey())
	{
		s3eKeyboardUpdate();
		IwGxClear(IW_GX_COLOUR_BUFFER_F | IW_GX_DEPTH_BUFFER_F);

		simLoop(0);

		IwGxFlush();
		IwGxSwapBuffers();
		s3eDeviceYield(0);
	}
}

//------------------------------------------------------------------------------
// Simulation loop is called and executed each simlation loop.
// dynamics is calculated by dWorldStep().
static void simLoop (int pause)
{
	const dReal *pos, *R;
	 
	dSpaceCollide (space,0, &nearCallback); // Collision decision, most first write this.
	dWorldStep (world, 0.1f); // Simulation 1 step is advanced
	dJointGroupEmpty (contactgroup); // The joint group is designated as the sky
	pos = dBodyGetPosition (ball.body); // Position
	R = dBodyGetRotation (ball.body); // Revolution queue
	dsDrawSphere (pos, R , radius); // Drawing of sphere
}
 
void START ()
{ 
	//setting of camera
	static float xyz [ 3 ] = {0.0, -3.0,1.0};  // Position of a view point [m]
	static float hpr [ 3 ] = {90.0,0.0,0.0};   // Direction of a gaze [°]
	dsSetViewpoint (xyz, hpr);  // Setting of the view point and the gaze
}
 
//------------------------------------------------------------------------------
int main()
{
	// Init
	IwGxInit();
	START();

	dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
	dMass m1;
	 
	dInitODE();

	world = dWorldCreate ();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0); // Formation of contact group
	 
	dWorldSetGravity (world,0,0, -0.5);
	 
	// Create a ground
	ground = dCreatePlane (space,0,0,1,0);
	 
	// Create a ball
	ball.body = dBodyCreate (world);
	dMassSetZero (&m1);
	dMassSetSphereTotal (&m1, mass, radius);
	dBodySetMass (ball.body , &m1);
	dBodySetPosition (ball.body , x0, y0, z0);
	 
	ball.geom = dCreateSphere (space, radius); // Formation of ball geometry
	dGeomSetBody (ball.geom, ball.body); // Setting geom to body
	 
	dsSimulationLoop();
	
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
 
	// terminate
	IwGxTerminate();

	return 0;
}



