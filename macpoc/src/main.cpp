
#include <iostream>
#include <vector>


#include "SnippetCamera.hpp"
#include "SnippetRender.hpp"

#include "htree/Golden.hpp"
#include "hsim/Physics.hpp"
#include "hsim/Project.hpp"

#include "hsim/PxEngine.hpp"

#define RENDER_SNIPPET

/*
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
*/

/*
void keyPress(unsigned char key, const PxTransform& camera)
{
  switch(toupper(key))
  {
  case 'B':  createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);            break;
  case ' ':  createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);  break;
  }
}
*/
#ifdef RENDER_SNIPPET
//extern void initPhysics(bool interactive);
//extern void stepPhysics(bool interactive);
//extern void cleanupPhysics(bool interactive);
//extern void keyPress(unsigned char key, const PxTransform& camera);


namespace
{
hsim::PxEngine *sEngine;
std::unique_ptr<hsim::Iteration> sIteration;

Snippets::Camera*  sCamera;

void motionCallback(int x, int y)
{
  sCamera->handleMotion(x, y);
}



void keyboardCallback(unsigned char key, int x, int y)
{
  if(key==27)
    exit(0);
  
  if (key == '\r') {
    sIteration->Next();
  }
  
  if (key == 'r') {
    sIteration->Retry();
  }

  if(!sCamera->handleKey(key, x, y, 0.1))
    ;//keyPress(key, sCamera->getTransform());
  
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
 
  hsim::IterationStatus status = sIteration->Step();
  if (status == hsim::IterationStatus::kFailed) {
    sIteration->Next();
  }
  
  Snippets::startRender(sCamera->getEye(), sCamera->getDir());

  physx::PxScene* scene;
  PxGetPhysics().getScenes(&scene,1);
  physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC);
  if(nbActors)
  {
    std::vector<physx::PxRigidActor*> actors(nbActors);
    scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<physx::PxActor**>(&actors[0]), nbActors);
    Snippets::renderActors(&actors[0], static_cast<physx::PxU32>(actors.size()), true);
  }

  Snippets::finishRender();
}

void exitCallback(void)
{
  delete sCamera;
  delete sEngine;
}
}

void renderLoop()
{
  

  sEngine = new hsim::PxEngine();
  sIteration.reset(new hsim::Iteration(*sEngine));
  sIteration->Next();
  
  
  sCamera = new Snippets::Camera(physx::PxVec3(2.0f, 2.0f, 2.0f), physx::PxVec3(-0.6f,-0.2f,-0.7f));

  Snippets::setupDefaultWindow("PhysX Snippet HelloWorld");
  Snippets::setupDefaultRenderState();

  glutIdleFunc(idleCallback);
  glutDisplayFunc(renderCallback);
  glutKeyboardFunc(keyboardCallback);
  glutMouseFunc(mouseCallback);
  glutMotionFunc(motionCallback);
  motionCallback(0,0);

  atexit(exitCallback);

  glutMainLoop();
}
#endif

int main(int argc, char *argv[])
{
  
#ifdef RENDER_SNIPPET
  renderLoop();
#else
  int solutions = 0;
  int attempts = 0;
  auto engine = new hsim::PxEngine();
  std::unique_ptr<hsim::Iteration> iteration(new hsim::Iteration(*engine));
  iteration->Next();
  while (solutions < 10) {
    hsim::IterationStatus status = iteration->Step();
    if (status == hsim::IterationStatus::kComplete) {
      std::cout << "Found solution " << solutions << " for " << attempts << " attempts." << std::endl;
      solutions++;
      attempts++;
      iteration->Next();
    } else if (status == hsim::IterationStatus::kFailed) {
      attempts++;
      iteration->Next();
    }
  }
  
#endif
}
