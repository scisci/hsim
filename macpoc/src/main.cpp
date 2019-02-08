
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

static hsim::Handness handness = hsim::Handness::kRight;
  

hsim::PxEngine *sEngine;
bool requested_path = false;
std::vector<unsigned char> pixel_data;
std::string save_path;
std::unique_ptr<hsim::Iteration> sIteration;

GLuint FramebufferName;
GLuint RenderbufferName;
GLuint DepthbufferName;
Snippets::Camera*  sCamera;

void motionCallback(int x, int y)
{
  sCamera->handleMotion(x, y);
}


  void renderToTexture(Snippets::ProjType proj, const physx::PxVec3& eye, const physx::PxVec3& dir, const std::string& path)
{
  glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
  glViewport(0, 0, 512, 512);
  
  Snippets::startRender(proj, eye, dir, 0.1f, 10000.0f, handness);
  
  physx::PxScene* scene;
  PxGetPhysics().getScenes(&scene,1);
  physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC);
  if(nbActors)
  {
    std::vector<physx::PxRigidActor*> actors(nbActors);
    scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<physx::PxActor**>(&actors[0]), nbActors);
    Snippets::renderActors(&actors[0], static_cast<physx::PxU32>(actors.size()), false, *sIteration.get());
  }
  
  //glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, 512, 512, GL_BGR, GL_UNSIGNED_BYTE, &pixel_data[0]);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  std::ofstream image_file;
  image_file.open(path);
  image_file.put(0);
  image_file.put(0);
  image_file.put(2);
  image_file.put(0); image_file.put(0);
  image_file.put(0); image_file.put(0);
  image_file.put(0);
  image_file.put(0); image_file.put(0);
  image_file.put(0); image_file.put(0);
  image_file.put(512 & 0xFF);
  image_file.put((512 >> 8) & 0xFF);
  image_file.put(512 & 0xFF);
  image_file.put((512 >> 8) & 0xFF);
  image_file.put(24);
  image_file.put(0);
  image_file.write((const char *)&pixel_data[0], pixel_data.size());
  image_file.close();
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
    TestFile::OpenDirectory([](std::string path) {
      sIteration->Write(path);
    });
  }
  
  if (key == 'o') {
    TestFile::OpenFile([](std::string path) {
      sIteration->Open(path);
    });
  }
  
  if (key == 'p') {
    TestFile::OpenDirectory([](std::string path) {
      renderToTexture(Snippets::kPersp, physx::PxVec3(2.0, 2.0, 3.0), physx::PxVec3(-0.5, -0.25, -1.0), path + "/quarter.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(0.5, 1.0, 2.0), physx::PxVec3(0, 0, -1), path + "/front.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(0.5, 1.0, -2.0), physx::PxVec3(0, 0, 1), path + "/back.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(-2.0, 1.0, -0.5), physx::PxVec3(1, 0, 0), path + "/left.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(2.0, 1.0, -0.5), physx::PxVec3(-1, 0, 0), path + "/right.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(0.5, 4.0, -0.5), physx::PxVec3(0, -1, 0), path + "/top.tga");
      renderToTexture(Snippets::kOrtho, physx::PxVec3(0.5, -2.0, -0.5), physx::PxVec3(0, 1, 0), path + "/bottom.tga");
    });
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
  
  
  Snippets::startRender(Snippets::kPersp, sCamera->getEye(), sCamera->getDir(), 0.1f, 10000.0f, handness);
  
  // Draw axes
  
  glBegin(GL_LINES);
  glColor4f(1.0, 0.0f, 0.0f, 1.0f);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(1.0, 0.0, 0.0);
  glColor4f(0.0, 1.0f, 0.0f, 1.0f);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 1.0, 0.0);
  glColor4f(0.0, 0.0f, 1.0f, 1.0f);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 1.0);
  glEnd();
  
  physx::PxScene* scene;
  PxGetPhysics().getScenes(&scene,1);
  physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC);
  if(nbActors)
  {
    std::vector<physx::PxRigidActor*> actors(nbActors);
    scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<physx::PxActor**>(&actors[0]), nbActors);
    Snippets::renderActors(&actors[0], static_cast<physx::PxU32>(actors.size()), true, *sIteration.get());
  }
  
  
  for (auto& curve : sIteration->curves) {
    glBegin(GL_LINE_STRIP);
  
    for (int i = curve.start_index; i < curve.end_index; ++i) {
      auto& vertex = sIteration->curve_verts[i];
      glVertex3f(vertex.x(), vertex.y(), vertex.z());
    }
    glEnd();
  }


  
  
  //Snippets::renderActor(*cone_test.cone1.get());
  //Snippets::renderActor(*cone_test.cone2.get());
  
  
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
  
  
  
  
  sCamera = new Snippets::Camera(physx::PxVec3(2.0f, 1.0f, 5.0f), physx::PxVec3(-1.0f, -1.0f, -1.0f), handness);

  Snippets::setupDefaultWindow("PhysX Snippet HelloWorld");
  Snippets::setupDefaultRenderState(handness);
  
  pixel_data.resize(3 * 512 * 512);
  GLenum status;
  glGenFramebuffers(1, &FramebufferName);
  glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
  glGenRenderbuffers(1, &RenderbufferName);
  glBindRenderbuffer(GL_RENDERBUFFER, RenderbufferName);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, 512, 512);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RenderbufferName);
  status = glGetError();
  glGenRenderbuffers(1, &DepthbufferName);
  glBindRenderbuffer(GL_RENDERBUFFER, DepthbufferName);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 512, 512);
  status = glGetError();
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, DepthbufferName);

  status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    std::cout << "frame buffer failed " << status << std::endl;
  }
  
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

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
