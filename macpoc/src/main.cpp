
#include <iostream>
#include <vector>


#include "SnippetCamera.hpp"
#include "SnippetRender.hpp"

#include "htree/Golden.hpp"
#include "hsim/Physics.hpp"
#include "hsim/Project.hpp"

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

hsim::Matrix4 frust(
  hsim::Real left, hsim::Real right, hsim::Real bottom, hsim::Real top, hsim::Real znear, hsim::Real zfar)
{
    hsim::Real temp, temp2, temp3, temp4;
    temp = 2.0 * znear;
    temp2 = right - left;
    temp3 = top - bottom;
    temp4 = zfar - znear;
    hsim::Matrix4 mat;
    mat <<
      hsim::Vector4(temp / temp2, 0.0, 0.0, 0.0),
      hsim::Vector4(0.0, temp / temp3, 0.0, 0.0),
      hsim::Vector4((right + left) / temp2, (top + bottom) / temp3, (-zfar - znear) / temp4, -1.0),
      hsim::Vector4(0.0, 0.0, (-temp * zfar) / temp4, 0.0);
  
    return mat;
}

hsim::Matrix4 persp(
  hsim::Real fov_deg, hsim::Real aspect, hsim::Real znear, hsim::Real zfar)
{
  hsim::Real ymax, xmax;
  ymax = znear * tanf(fov_deg * M_PI / 360.0);
  xmax = ymax * aspect;
  return frust(-xmax, xmax, -ymax, ymax, znear, zfar);
  
}

physx::PxMat44 ConvertTransform(const hsim::Matrix4& matrix)
  {
    float values[16];
    for (int i = 0; i < 16; i++) {
      values[i] = matrix(i);
    }
    return physx::PxMat44(values);
  }

hsim::Matrix4 CalcViewMatrix(hsim::Vector3 eye, hsim::Vector3 dir)
{
  hsim::Vector3 z_axis = dir.normalized();
  hsim::Vector3 x_axis = z_axis.cross(hsim::Vector3(0, 1, 0)).normalized();
  hsim::Vector3 y_axis = x_axis.cross(z_axis).normalized();

  hsim::Matrix4 orientation;
  orientation <<
    hsim::Vector4(x_axis.x(), y_axis.x(), -z_axis.x(), 0.0),
    hsim::Vector4(x_axis.y(), y_axis.y(), -z_axis.y(), 0.0),
    hsim::Vector4(x_axis.z(), y_axis.z(), -z_axis.z(), 0.0),
    hsim::Vector4(0, 0, 0, 1);
  
  hsim::Matrix4 trans = hsim::Matrix4::Identity();
  trans(12) = -eye.x();
  trans(13) = -eye.y();
  trans(14) = -eye.z();
  
  return orientation * trans;

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

template<class T>
Eigen::Matrix<T,4,4> lookAt
(
    Eigen::Matrix<T,3,1> const & eye,
    Eigen::Matrix<T,3,1> const & center,
    Eigen::Matrix<T,3,1> const & up
)
{
    typedef Eigen::Matrix<T,4,4> Matrix4;
    typedef Eigen::Matrix<T,3,1> Vector3;
    

    Vector3 f = (center - eye).normalized();
    Vector3 u = up.normalized();
    Vector3 s = f.cross(u).normalized();
    u = s.cross(f);

    Matrix4 res;
    res <<  s.x(),s.y(),s.z(),0,//-s.dot(eye),
            u.x(),u.y(),u.z(),0,//-u.dot(eye),
            -f.x(),-f.y(),-f.z(),0,//f.dot(eye),
            0,0,0,1;
  
    Matrix4 o;
    o <<  0, 0, 0, 0,//-s.dot(eye),
          0, 1, 0, 0,//-u.dot(eye),
          0, 0, 1, 0,//f.dot(eye),
          0, 0, 0 , 1;

    return o * res;
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
  


  hsim::Matrix4 view_matrix = CalcViewMatrix(hsim::Vector3(eye.x, eye.y, eye.z), hsim::Vector3(dir.x, dir.y, dir.z));
  hsim::Matrix4 projection_matrix = persp(fovy, width / height, znear, zfar);
  hsim::Matrix4 window_matrix =
    (Eigen::Scaling(0.5 * width, 0.5 *height, 0.5) *
    Eigen::Translation<hsim::Real, 3>(1.0, 1.0, 1.0)).matrix();
  //hsim::Matrix4 temp;
  //temp.matrix() = window_matrix.matrix();
  //print_mat4(temp);

 // hsim::Vector4 test_point = view_matrix.inverse() * hsim::Vector4(0.0, 0.0, -10.0, 1.0);
  
  hsim::Matrix4 mvpw = (projection_matrix * view_matrix).inverse();//(window_matrix * projection_matrix * view_matrix);
  physx::PxMat44 pxproj = ConvertTransform(projection_matrix);
  physx::PxMat44 pxview = ConvertTransform(view_matrix);
  physx::PxMat44  pxmvpw(physx::PxTransform(pxproj * pxview).getInverse());

  
  hsim::Vector4 start = hsim::Vector4(
    (2.0 * x) / width - 1,
    1.0 - (2.0 * y) / height,
    -1.0, 1.0);
  
  
      hsim::Real dist = -1.0 + 20.0 * (2.0 / (zfar - znear));
  
  hsim::Vector4 end = hsim::Vector4(
    (2.0 * x) / width - 1,
    1.0 - (2.0 * y) / height,
    1.0, 1.0);
  /*
  hsim::Vector4 ray_clip = hsim::Vector4(ray_device.x(), ray_device.y(), -1.0, 1.0);
  hsim::Vector4 ray_eye = projection_matrix.inverse() * ray_clip;
  ray_eye.z() = -1.0;
  ray_eye.w() = 0.0;
  ray_eye = view_matrix.inverse() * ray_eye;
  */
  hsim::Vector4 ray_start = mvpw * start;
  
  hsim::Vector4 ray_end = mvpw * end;
  ray_start /= ray_start.w();
  ray_end /= ray_end.w();
  
  hsim::Vector4 pos = ray_start + (ray_end - ray_start).normalized() * 20.0;
  std::cout << "ray goes from " <<
    ray_start.x() << ", " << ray_start.y() << ", " << ray_start.z() << " to \n" <<
    ray_end.x() << ", " << ray_end.y() << ", " << ray_end.z() << std::endl;
  std::cout << " click " << x << ", " << y << " near ( " << pos.x() << ", " << pos.y() << ", " << pos.z() << " )" << std::endl;
  sIteration->AddMouseTest(hsim::Vector3(pos.x(), pos.y(), pos.z()));

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
  
  
  sCamera = new Snippets::Camera(physx::PxVec3(0.0f, 2.0f, 2.0f), physx::PxVec3(0.0f, 0.0f, -1.0f));

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
