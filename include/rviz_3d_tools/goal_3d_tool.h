#pragma once

#include <rviz/tool.h>
#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <OgreVector3.h>

namespace rviz_3d_tools {

class Goal3DTool : public rviz::Tool {
  Q_OBJECT
public:
  Goal3DTool();
  ~Goal3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

private:
  bool get3DPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& out);
  bool getDirectionPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& out);

  ros::Publisher pub_;
  rviz::Arrow* arrow_;

  bool got_position_;
  Ogre::Vector3 pos_;
};

} // namespace rviz_3d_tools