//
// Bullet Test
//

#include "lib/framework.hpp"
#include <cassert>
#include <iostream>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>


#if defined (_MSC_VER)
// ライブラリをリンクする
#if defined(DEBUG)
#pragma comment(lib, "Debug/BulletCollision_Debug.lib")
#pragma comment(lib, "Debug/BulletDynamics_Debug.lib")
#pragma comment(lib, "Debug/LinearMath_Debug.lib")
#else
#pragma comment(lib, "Release/BulletCollision.lib")
#pragma comment(lib, "Release/BulletDynamics.lib")
#pragma comment(lib, "Release/LinearMath.lib")
#endif
#endif


enum Size
{
  WIDTH = 640,
  HEIGHT = 960
};


glm::vec4 convVec(const btVector3& p)
{
  return { p.x(), p.y(), p.z(), p.w() };
}


// デバッグ用表示
class MyDebugDraw : public btIDebugDraw {
  int debug_mode;
  glm::mat4 proj_view_;


  void drawLine(const btVector3& from, const btVector3& to,
                const btVector3& color) {
    auto f = proj_view_ * convVec(from);
    auto t = proj_view_ * convVec(to);

    ::drawLine(f.x, f.y, t.x, t.y, 1, Color(color.x(), color.y(), color.z()));
  }

  void drawLine(const btVector3& from, const btVector3& to,
                const btVector3& fromColor, const btVector3& toColor) {
    auto f = proj_view_ * convVec(from);
    auto t = proj_view_ * convVec(to);

    ::drawLine(f.x, f.y, t.x, t.y, 1, Color(fromColor.x(), fromColor.y(), fromColor.z()));
  }

  void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
                        btScalar distance, int lifeTime,
                        const btVector3& color) {

    /* PointOnBに、衝突点を描画 */
    // ::drawPoint(PointOnB.x(), PointOnB.y(), 2, Color(color.x(), color.y(), color.z()));
  }


  void reportErrorWarning(const char* warningString) {

    /* 警告表示 */

  }

  void draw3dText(const btVector3& location, const char* textString) {

    /* 指定空間座標に文字列表示 */

  }



public:
  void setDebugMode(int debugMode) {

    /* デバッグモード指定 */
    debug_mode = debugMode;

  }

  int getDebugMode() const {

    /* 現在のデバッグモードを返却 */
    return debug_mode;
  }

  void setProjViewMatrix(const glm::mat4& mat)
  {
    proj_view_ = mat;
  }

};



std::pair<btMotionState*, btRigidBody*> AddRigidBody(btDynamicsWorld* world, btCollisionShape* shape, btScalar mass, const btTransform& transform)
{
  bool isDynamic = mass > 0.0f;

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
  {
    shape->calculateLocalInertia(mass, localInertia);
  }

  auto* state = new btDefaultMotionState(transform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, state, shape, localInertia);

  // 抵抗(値が大きいほど抵抗が強い)
  rbInfo.m_linearDamping = 0.1f;
  rbInfo.m_angularDamping = 0.1f;
  // 摩擦係数(大きいほど摩擦力が強い)
  rbInfo.m_friction = 0.2f;
  rbInfo.m_rollingFriction = 0.2f;
  rbInfo.m_spinningFriction = 0.2f;
  // 反発力
  rbInfo.m_restitution = 0.5;
  // RigidBodyが「止まった」と判定される閾値
  rbInfo.m_linearSleepingThreshold = 0.01;
  rbInfo.m_angularSleepingThreshold = 0.01;

  auto* body = new btRigidBody(rbInfo);
  world->addRigidBody(body);

  return  { state, body };
}

std::pair<btMotionState*, btRigidBody*> addSphere(btDynamicsWorld* world, float radius, const btVector3& pos, float mass)
{
  auto* shape = new btSphereShape(radius);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);

  return AddRigidBody(world, shape, mass, transform);
}

std::pair<btMotionState*, btRigidBody*> addBox(btDynamicsWorld* world, const btVector3& size, const btVector3& pos, float mass)
{
  auto* shape = new btBoxShape(size);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);

  return AddRigidBody(world, shape, mass, transform);
}

std::pair<btMotionState*, btRigidBody*> addCapsule(btDynamicsWorld* world, float radius, float height, const btVector3& pos, const btVector3& rot, float mass)
{
  auto* shape = new btCapsuleShape(radius, height);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);
  transform.setRotation(btQuaternion(rot.x(), rot.y(), rot.z()));

  return AddRigidBody(world, shape, mass, transform);
}

std::pair<btMotionState*, btRigidBody*> addStaticPlane(btDynamicsWorld* world, const btVector3& up, const btVector3& pos, float mass)
{
  auto* shape = new btStaticPlaneShape(up, 0);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);

  return AddRigidBody(world, shape, mass, transform);
}



//
// メインプログラム
//
int main()
{
  AppEnv env(Size::WIDTH, Size::HEIGHT);


  // Bullet 初期化
  auto* config = new btDefaultCollisionConfiguration();
  auto* dispatcher = new btCollisionDispatcher(config);
  auto* broadphase = new btDbvtBroadphase();
  auto* solver = new btSequentialImpulseConstraintSolver();
  auto* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);

  world->setGravity(btVector3(0, -98, 0));

  // 地面
  addBox(world, btVector3(100, 25, 100), btVector3(0, -300 - 30, 0), 0);
  addStaticPlane(world, btVector3(0, 1, 0), btVector3(0, -300, 0), 0);
  addStaticPlane(world, btVector3(1, 0, 0), btVector3(-200, 0, 0), 0);
  addStaticPlane(world, btVector3(-1, 0, 0), btVector3(200, 0, 0), 0);
  addStaticPlane(world, btVector3(0, 0, 1), btVector3(0, 0, -200), 0);
  addStaticPlane(world, btVector3(0, 0, -1), btVector3(0, 0, 200), 0);

  auto sphere = addSphere(world, 20, btVector3(0, 0, 0), 10);
  sphere.second->setActivationState(DISABLE_DEACTIVATION);
  auto* sphere_constraint = new btPoint2PointConstraint(*sphere.second, btVector3(0, 10, 0));
  world->addConstraint(sphere_constraint);

  float pos_y = 0 - 21;

  // 紐
  float height = 10;
  float radius = 5;

  std::vector<std::pair<btMotionState*, btRigidBody*>> objects; 

  pos_y -= radius + height * 0.5 + 0.5;

  for (int i = 0; i < 20; ++i)
  {
    auto obj = addCapsule(world, radius, height, btVector3(0, pos_y, 0), btVector3(0, 0, 0), 1);
    obj.second->setActivationState(DISABLE_DEACTIVATION);
    pos_y -= radius * 2 + height + 2.0;
    objects.push_back(obj);
  }

  {
    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(btVector3(0.0f, -21.0f, 0.0f));

    btTransform t2;
    t2.setIdentity();
    t2.setOrigin(btVector3(0.0f, radius + height * 0.5f + 0.5f, 0.0f));

    auto* constraint = new btGeneric6DofConstraint(*sphere.second, *objects[0].second, t1, t2, true);
    // constraint->enableSpring(1, true);
    // constraint->setStiffness(1, 0.5f);
    // constraint->setDamping(1, 0.5f);
    // constraint->enableSpring(4, true);
    // constraint->setStiffness(4, 0.5f);
    // constraint->setDamping(4, 0.5f);

    world->addConstraint(constraint);
  }


  for (int i = 0; i < objects.size() - 1; ++i)
  {
    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(btVector3(0.0f, -radius + height * -0.5f - 1.8f, 0.0f));

    btTransform t2;
    t2.setIdentity();
    t2.setOrigin(btVector3(0.0f, radius + height * 0.5f + 1.8f, 0.0f));

    // auto* constraint = new btGeneric6DofSpringConstraint(*objects[i].second, *objects[i + 1].second, t1, t2, true);
    auto* constraint = new btGeneric6DofConstraint(*objects[i].second, *objects[i + 1].second, t1, t2, true);
    // constraint->setLimit(0, 0.0f, 0.0f);
    // constraint->setLimit(1, 0.0f, 0.0f);
    // constraint->setLimit(2, 0.0f, 0.0f);
    // constraint->setLimit(3, 0.0f, 0.0f);
    // constraint->setLimit(4, 0.0f, 0.0f);
    // constraint->setLimit(5, 0.0f, 0.0f);
    constraint->setLinearLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
    constraint->setLinearUpperLimit(btVector3(0.0f, 2.0f, 0.0f));
    constraint->setAngularLowerLimit(btVector3(-0.2, -0.2, -0.2));
    constraint->setAngularUpperLimit(btVector3(0.2, 0.2, 0.2));
    // constraint->setParam(BT_CONSTRAINT_CFM, 0);
    // constraint->setParam(BT_CONSTRAINT_STOP_CFM, 0);
    // constraint->setParam(BT_CONSTRAINT_ERP, 0.1);
    // constraint->setParam(BT_CONSTRAINT_STOP_ERP, 0.1);

    // constraint->enableSpring(1, true);
    // constraint->setStiffness(1, 350.0f);
    // constraint->setDamping(1, 0.5f);

    constraint->setDbgDrawSize(20.0f);

    world->addConstraint(constraint);
    
  }

  // 先端の物体
  {
    float mass = 50;
    auto bottom_obj = addCapsule(world, 20, 10, btVector3(0, pos_y, 0), btVector3(0, 0, 0), mass);
    auto* body = bottom_obj.second;
    auto* shape = body->getCollisionShape();
    btVector3 localInertia(0, -10, 0);
    shape->calculateLocalInertia(mass, localInertia);
    body->setActivationState(DISABLE_DEACTIVATION);

    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(btVector3(0.0f, -radius + height * -0.5f - 0.8f, 0.0f));

    btTransform t2;
    t2.setIdentity();
    t2.setOrigin(btVector3(15, 20, 0.0f));

    auto* constraint = new btGeneric6DofConstraint(*objects[objects.size() - 1].second, *bottom_obj.second, t1, t2, true);
    // constraint->enableSpring(1, true);
    // constraint->setStiffness(1, 0.5f);
    // constraint->setDamping(1, 0.5f);
    // constraint->enableSpring(4, true);
    // constraint->setStiffness(4, 0.5f);
    // constraint->setDamping(4, 0.5f);

    world->addConstraint(constraint);
  }

  {
    Random rand;
    for (int i = 0; i < 25; ++i)
    {
      float x = rand(-100, 100);
      float y = rand(200, 400);
      float z = rand(-100, 100);
      auto sphere = addSphere(world, 20, btVector3(x, y, z), 5);
    }
  }



  // デバッグ表示
  auto* debug_draw = new MyDebugDraw();

  auto proj = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
  auto view = glm::lookAt(glm::vec3(-160, 120, -160), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
  auto mat = proj * view;
  debug_draw->setProjViewMatrix(mat);

  debug_draw->setDebugMode(btIDebugDraw::DBG_DrawWireframe
                           | btIDebugDraw::DBG_DrawContactPoints
                           | btIDebugDraw::DBG_DrawConstraints);

  world->setDebugDrawer(debug_draw);

  env.bgColor(Color(0.5, 0.5, 0.5));
  while (env.isOpen())
  {
    env.begin();

    auto pos = env.mousePosition();
    sphere_constraint->setPivotB(btVector3(pos.x, pos.y, 0));

    // 時間を進める
    world->stepSimulation(1.0f / 60.0f, 10);
    // デバグ表示
    world->debugDrawWorld();

    env.end();
  }
}
