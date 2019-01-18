
#include <iostream>
#include <vector>


#include "SnippetCamera.hpp"
#include "SnippetRender.hpp"

#include "htree/Golden.hpp"
#include "hsim/Physics.hpp"
#include "hsim/Project.hpp"
#include "hsim/Math.hpp"

#include "hsim/PxEngine.hpp"

#include "FileUtils.hpp"


#define RENDER_SNIPPET



/*
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
bool requested_path = false;
std::string save_path;
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
  
  if (key == 'w') {
    // Write
    sIteration->Write();
  }

  if(!sCamera->handleKey(key, x, y, 0.1))
    ;//keyPress(key, sCamera->getTransform());
  
}






physx::PxMat44 ConvertTransform(const hsim::Matrix4& matrix)
  {
    float values[16];
    for (int i = 0; i < 16; i++) {
      values[i] = matrix(i);
    }
    return physx::PxMat44(values);
  }


template<class T>
Eigen::Matrix<T,4,4> perspective
(
    double fovy,
    double aspect,
    double zNear,
    double zFar
)
{
    typedef Eigen::Matrix<T,4,4> Matrix4;

    assert(aspect > 0);
    assert(zFar > zNear);

    double radf = fovy * M_PI / 180.0;

    double tanHalfFovy = tan(radf / 2.0);
    Matrix4 res = Matrix4::Zero();
    res(0,0) = 1.0 / (aspect * tanHalfFovy);
    res(1,1) = 1.0 / (tanHalfFovy);
    res(2,2) = - (zFar + zNear) / (zFar - zNear);
    res(3, 2) = - 1.0;
    res(2, 3) = - (2.0 * zFar * zNear) / (zFar - zNear);
    return res;
}

void print_mat4(const hsim::Matrix4 mat)
{
  std::cout <<
    mat(0, 0) << " " << mat(0, 1) << " " << mat(0, 2) << " " << mat(0, 3) << "\n" <<
    mat(1, 0) << " " << mat(1, 1) << " " << mat(1, 2) << " " << mat(1, 3) << "\n" <<
    mat(2, 0) << " " << mat(2, 1) << " " << mat(2, 2) << " " << mat(2, 3) << "\n" <<
    mat(3, 0) << " " << mat(3, 1) << " " << mat(3, 2) << " " << mat(3, 3) << std::endl;
}


void mouseCallback(int button, int state, int x, int y)
{
  sCamera->handleMouse(button, state, x, y);

  hsim::Real width = 512.0;
  hsim::Real height = 512.0;
  hsim::Real znear = 1.0f;
  hsim::Real zfar = 10000.0f;
  hsim::Real fovy = 60.0;
  physx::PxVec3 eye = sCamera->getEye();
  physx::PxVec3 dir = sCamera->getDir();
  physx::PxVec3 targ = eye + dir;
  


  hsim::Matrix4 view_matrix = hsim::CalcViewMatrix(
    hsim::Vector3(eye.x, eye.y, eye.z),
    hsim::Vector3(targ.x, targ.y, targ.z));
  hsim::Matrix4 proj_matrix = hsim::CalcPerspectiveProjection(
    fovy, width / height, znear, zfar);
  
  hsim::Ray ray = hsim::CastRayFor2DCoords(
    x, y, width, height, proj_matrix, view_matrix);

  
  hsim::Vector3 pos = ray.PointAtDistance(20.0);
  sIteration->AddMouseTest(pos);

}

void idleCallback()
{
  if (!requested_path) {
    requested_path = true;
    TestFile::OpenDirectory([](std::string path) {
      save_path = path;
    });
  }
  
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
  
  
  sCamera = new Snippets::Camera(physx::PxVec3(2.0f, 2.0f, 2.0f), physx::PxVec3(-1.0f, -1.0f, -1.0f));

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
