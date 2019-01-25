
#include <iostream>
#include <vector>


#include "SnippetCamera.hpp"
#include "SnippetRender.hpp"

#include "htree/Golden.hpp"
#include "hsim/Physics.hpp"
#include "hsim/Project.hpp"
#include "hsim/Math.hpp"

#include "hsim/RealRangeDichotomy.hpp"

#include "hsim/PxEngine.hpp"

#include "FileUtils.hpp"
#include "hsim/ParabolaMotionValidator.hpp"

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

struct ConeTest {
  ConeTest()
  {
    hsim::Real friction = 1.2;
    hsim::Real max_height = 7.0;
    hsim::Real max_vel = sqrt(max_height * 9.81);
    hsim::ParabolaMotionValidator validator(max_vel, friction);
    hsim::Real phi = hsim::ParabolaMotionValidator::ComputeHalfConeAngle(1.2);
    hsim::Real cone_height = 0.1;
    hsim::Real cone_radius = tan(phi) * cone_height;
    
    hsim::Vector3 cone1_pos(-3.0, 4.0, 0.0);
    hsim::Vector3 cone1_up(0.0, 1.0, 0.0);
    hsim::Vector3 cone2_pos(-1.0, 2.0, 0.0);
    hsim::Vector3 cone2_up(0.0, 1.0, 0.0);
    
    hsim::RigidBodyBuilder builder;
    hsim::Transform transform = hsim::Transform::Identity();
    // Rotate cone 90 degrees since by default opengl renders as point down z-axis

    transform.rotate(Eigen::AngleAxis<hsim::Real>(M_PI / 2.0, hsim::Vector3::UnitX()));
    transform.translation() = cone1_pos + hsim::Vector3(0, cone_height, 0);
    std::unique_ptr<hsim::Geometry> geom(new hsim::Cone(cone_radius, cone_height));
    builder.AddShape(std::move(geom), 1000.0, transform);
    cone1 = builder.Build();
    
    transform = hsim::Transform::Identity();
    transform.rotate(Eigen::AngleAxis<hsim::Real>(M_PI / 2.0, hsim::Vector3::UnitX()));
    transform.translation() = cone2_pos + hsim::Vector3(0, cone_height, 0);
    geom.reset(new hsim::Cone(cone_radius, cone_height));
    builder.AddShape(std::move(geom), 1000.0, transform);
    cone2 = builder.Build();
    
    // Calculate the curve
    
    auto path = validator.ComputePath(cone1_pos, cone1_up, cone2_pos, cone2_up);
    if (path == nullptr) {
      // no path
    } else {
      int num_points = 20;
      hsim::Real length = path->Length();
      for (int i = 0; i <= 20; ++i) {
        curve.push_back(path->Compute(i * length/20.0));
      }
      
      hsim::Vector3 last = path->Compute(0.99 * length);
      hsim::Vector3 dif = (last - cone2_pos).normalized();
      hsim::Real min = M_PI / 2 - phi;
      hsim::Real angle = M_PI - acos(dif.dot(hsim::Vector3::UnitX()));
      hsim::Real other = atan2(dif(1), dif(0));
      std::cout << "Angle is " << angle << " min is " << min << std::endl;
    }
  }
  
  std::unique_ptr<hsim::RigidBody> cone1;
  std::unique_ptr<hsim::RigidBody> cone2;
  std::vector<hsim::Vector3> curve;
};

hsim::PxEngine *sEngine;
bool requested_path = false;
std::string save_path;
std::unique_ptr<hsim::Iteration> sIteration;
ConeTest cone_test;

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








void mouseCallback(int button, int state, int x, int y)
{
  sCamera->handleMouse(button, state, x, y);

/*
  // Drop a box based on the mouse coord
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
*/
}

void idleCallback()
{
/*
  if (!requested_path) {
    requested_path = true;
    TestFile::OpenDirectory([](std::string path) {
      save_path = path;
    });
  }
  */
  
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
  
  glBegin(GL_LINE_STRIP);
  
  for (auto& vertex : cone_test.curve) {
    glVertex3f(vertex.x(), vertex.y(), vertex.z());
  }

glEnd();
  
  
  Snippets::renderActor(*cone_test.cone1.get());
  Snippets::renderActor(*cone_test.cone2.get());
  
  
  // Render the cone path
  
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
  
  
  sCamera = new Snippets::Camera(physx::PxVec3(2.0f, 1.0f, 5.0f), physx::PxVec3(-1.0f, -1.0f, -1.0f));

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
  hsim::RealRangeDichotomy d(10.0, 110.0, 10);
  for (auto& part : d) {
    std::cout << part;
  }

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
