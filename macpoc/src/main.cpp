
#include <iostream>
#include <vector>

#include "PxPhysicsAPI.h"
#include "SnippetCamera.hpp"
#include "SnippetRender.hpp"

#include "htree/Golden.hpp"
#include "hsim/Simulation.hpp"

using namespace physx;

#define PVD_HOST "127.0.0.1"  //Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.
#define RENDER_SNIPPET

PxDefaultAllocator    gAllocator;
PxDefaultErrorCallback  gErrorCallback;

PxFoundation*      gFoundation = NULL;
PxPhysics*        gPhysics  = NULL;

PxDefaultCpuDispatcher*  gDispatcher = NULL;
PxScene*        gScene    = NULL;

PxMaterial*        gMaterial  = NULL;

PxPvd*                  gPvd        = NULL;

PxReal stackZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
  PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
  dynamic->setAngularDamping(0.5f);
  dynamic->setLinearVelocity(velocity);
  gScene->addActor(*dynamic);
  return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
  PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
  for(PxU32 i=0; i<size;i++)
  {
    for(PxU32 j=0;j<size-i;j++)
    {
      PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
      PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
      body->attachShape(*shape);
      PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
      gScene->addActor(*body);
    }
  }
  shape->release();
}

void initPhysics(bool interactive)
{
  gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

  gPvd = PxCreatePvd(*gFoundation);
  PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
  gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

  gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

  PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
  gDispatcher = PxDefaultCpuDispatcherCreate(2);
  sceneDesc.cpuDispatcher  = gDispatcher;
  sceneDesc.filterShader  = PxDefaultSimulationFilterShader;
  gScene = gPhysics->createScene(sceneDesc);

  PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
  if(pvdClient)
  {
    pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
    pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
  }
  gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

  PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
  gScene->addActor(*groundPlane);

  for(PxU32 i=0;i<5;i++)
    createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

  if(!interactive)
    createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
}

void stepPhysics(bool interactive)
{
  PX_UNUSED(interactive);
  gScene->simulate(1.0f/60.0f);
  gScene->fetchResults(true);
}

void cleanupPhysics(bool interactive)
{
  PX_UNUSED(interactive);
  gScene->release();
  gDispatcher->release();
  gPhysics->release();
  PxPvdTransport* transport = gPvd->getTransport();
  gPvd->release();
  transport->release();
  
  gFoundation->release();
  
  printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
  switch(toupper(key))
  {
  case 'B':  createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);            break;
  case ' ':  createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);  break;
  }
}

#ifdef RENDER_SNIPPET
extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);


namespace
{
Snippets::Camera*  sCamera;

void motionCallback(int x, int y)
{
  sCamera->handleMotion(x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
  if(key==27)
    exit(0);

  if(!sCamera->handleKey(key, x, y))
    keyPress(key, sCamera->getTransform());
}

void mouseCallback(int button, int state, int x, int y)
{
  sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
  glutPostRedisplay();
}

void renderCallback()
{
  stepPhysics(true);

  Snippets::startRender(sCamera->getEye(), sCamera->getDir());

  PxScene* scene;
  PxGetPhysics().getScenes(&scene,1);
  PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
  if(nbActors)
  {
    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
    Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
  }

  Snippets::finishRender();
}

void exitCallback(void)
{
  delete sCamera;
  cleanupPhysics(true);
}
}

void renderLoop()
{
  sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f,-0.2f,-0.7f));

  Snippets::setupDefaultWindow("PhysX Snippet HelloWorld");
  Snippets::setupDefaultRenderState();

  glutIdleFunc(idleCallback);
  glutDisplayFunc(renderCallback);
  glutKeyboardFunc(keyboardCallback);
  glutMouseFunc(mouseCallback);
  glutMotionFunc(motionCallback);
  motionCallback(0,0);

  atexit(exitCallback);

  initPhysics(true);
  glutMainLoop();
}
#endif

int main(int argc, char *argv[])
{
  hsim::Simulation sim("johnny");
  
  htree::golden::GoldenRatioSource source;
  const htree::Ratios& ratios = source.Ratios();
  std::cout << ratios[0] << std::endl;
#ifdef RENDER_SNIPPET
  renderLoop();
#else
  static const PxU32 frameCount = 100;
  initPhysics(false);
  for(PxU32 i=0; i<frameCount; i++)
    stepPhysics(false);
  cleanupPhysics(false);
#endif
}
