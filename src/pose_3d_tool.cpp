#include "rviz_3d_tools/pose_3d_tool.h"

#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/geometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <OgreSceneManager.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreMatrix4.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreRenderTarget.h>

#include <GL/gl.h>

#include <cmath>

namespace rviz_3d_tools {

Pose3DTool::Pose3DTool()
  : arrow_(nullptr), got_position_(false)
{
  shortcut_key_ = 'p';
}

Pose3DTool::~Pose3DTool() {
  delete arrow_;
}

void Pose3DTool::onInitialize() {
  ros::NodeHandle nh;
  pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  arrow_ = new rviz::Arrow(scene_manager_, nullptr, 0.8f, 0.05f, 0.2f, 0.1f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate() {}

void Pose3DTool::deactivate() {
  arrow_->getSceneNode()->setVisible(false);
  got_position_ = false;
}

bool Pose3DTool::get3DPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& out) {
  // Read depth from the already-rendered framebuffer — no new render pass,
  // no SelectionManager involvement, so no Ogre render-queue corruption.
  Ogre::Camera*      cam    = event.viewport->getCamera();
  Ogre::RenderTarget* rt    = event.viewport->getTarget();

  int vp_left   = event.viewport->getActualLeft();
  int vp_top    = event.viewport->getActualTop();
  int vp_w      = event.viewport->getActualWidth();
  int vp_h      = event.viewport->getActualHeight();
  int rt_h      = static_cast<int>(rt->getHeight());

  // OpenGL origin is bottom-left; screen/RViz origin is top-left.
  int gl_x = vp_left + event.x;
  int gl_y = rt_h - (vp_top + event.y) - 1;

  // Read from the front buffer (the last completed frame visible to the user).
  GLint prev_read = GL_BACK;
  glGetIntegerv(GL_READ_BUFFER, &prev_read);
  glReadBuffer(GL_FRONT);

  GLfloat depth = 1.0f;
  glReadPixels(gl_x, gl_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  GLenum err = glGetError();

  glReadBuffer(prev_read);

  if (err == GL_NO_ERROR && depth < 1.0f - 1e-4f) {
    // Back-project: screen (x,y) + depth → world position.
    float ndcX =  2.0f * static_cast<float>(event.x) / vp_w - 1.0f;
    float ndcY =  1.0f - 2.0f * static_cast<float>(event.y) / vp_h;
    float ndcZ =  depth * 2.0f - 1.0f;   // OpenGL NDC depth: [0,1] → [-1,1]

    Ogre::Matrix4 VP    = cam->getProjectionMatrixWithRSDepth() * cam->getViewMatrix();
    Ogre::Matrix4 VPinv = VP.inverse();

    Ogre::Vector4 clip(ndcX, ndcY, ndcZ, 1.0f);
    Ogre::Vector4 world = VPinv * clip;

    if (std::fabs(static_cast<float>(world.w)) > 1e-6f) {
      world /= world.w;
      out = Ogre::Vector3(world.x, world.y, world.z);
      return true;
    }
  }

  // Fallback: intersect camera ray with the horizontal plane at z = 0.
  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  return rviz::getPointOnPlaneFromWindowXY(event.viewport, ground, event.x, event.y, out);
}

bool Pose3DTool::getDirectionPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& out) {
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, pos_.z);
  return rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, out);
}

int Pose3DTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  // rviz::Arrow points along local +X; rotate -90° around Y to align with ROS heading.
  static const Ogre::Quaternion orient_x(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  if (!got_position_) {
    if (event.leftDown()) {
      if (get3DPoint(event, pos_)) {
        got_position_ = true;
        arrow_->setPosition(pos_);
        arrow_->getSceneNode()->setVisible(true);
      }
    }
    return Render;
  }

  Ogre::Vector3 cur = pos_;
  getDirectionPoint(event, cur);

  Ogre::Vector3 dir = cur - pos_;
  if (dir.squaredLength() > 1e-6f) {
    double yaw = std::atan2(dir.y, dir.x);
    Ogre::Quaternion q;
    q.FromAngleAxis(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z);
    arrow_->setOrientation(q * orient_x);
  }

  if (event.leftUp()) {
    double yaw = std::atan2(static_cast<double>(dir.y), static_cast<double>(dir.x));

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = context_->getFixedFrame().toStdString();

    msg.pose.pose.position.x = pos_.x;
    msg.pose.pose.position.y = pos_.y;
    msg.pose.pose.position.z = pos_.z;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

    msg.pose.covariance[0]  = 0.25;
    msg.pose.covariance[7]  = 0.25;
    msg.pose.covariance[35] = 0.068;

    pub_.publish(msg);
    ROS_INFO("[Pose3DTool] Published initial pose: (%.2f, %.2f, %.2f) yaw=%.2f",
             pos_.x, pos_.y, pos_.z, yaw);

    arrow_->getSceneNode()->setVisible(false);
    got_position_ = false;
    return Render | Finished;
  }

  return Render;
}

} // namespace rviz_3d_tools

PLUGINLIB_EXPORT_CLASS(rviz_3d_tools::Pose3DTool, rviz::Tool)
