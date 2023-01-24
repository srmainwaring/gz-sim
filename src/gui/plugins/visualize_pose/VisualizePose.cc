/*
 * Copyright (C) 2023 Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/**
  \todo(srmainwaring) features in RViz

  Pose
    Topic: topic string
      Depth: depth of incoming message queue
      History Policy: System Default, Keep Last, Keep All.
      Reliability Policy: System Default, Reliable, Best Effort.
      Durability Policy: System Default, Transient Local, Volatile.
      Filter size: 10
    Shape: Arrow, Axes
      Arrow:
        Color: 255, 25, 0
        Alpha: 1
        Shaft Length: 1
        Shaft Radius: 0.05
        Head Length: 0.3
        Head Radius: 0.1
      Axes:
        Axes Length: 1
        Axes Radius: 0.1

**/


#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/Util.hh"

#include "VisualizePose.hh"

namespace gz
{
namespace sim
{
namespace gui
{

/// \brief Private data class for VisualizePose
class VisualizePosePrivate
{
  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Whether currently locked on a given entity
  public: bool locked{false};

  /// \brief Current target entity.
  public: Entity targetEntity{kNullEntity};

  /// \brief Name of the target entity.
  public: std::string targetName;

  /// \brief Whether the target entity has been changed.
  public: bool targetEntityDirty{false};

  /// \brief Whether the target name has been changed.
  public: bool targetNameDirty{false};

  /// \brief Marker messages.
  public: msgs::Marker markerMsg;
  public: msgs::Marker* markerMsg1{nullptr};
  public: msgs::Marker* markerMsg2{nullptr};
  public: msgs::Marker* markerMsg3{nullptr};
  public: msgs::Marker* markerMsg4{nullptr};
  public: msgs::Marker_V markerMsgs;

  /// \brief Maximum number of poses to display. Older poses are deleted.
  public: int maxPoints{50};

  /// \brief Whether the max points has been changed.
  public: bool maxPointsDirty{false};

  /// \brief Current marker ID. Calculated modulo maxPoints.
  public: int markerID{0};

  /// \brief Time between marker updates (secs).
  public: std::chrono::steady_clock::duration updatePeriod{
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::seconds{1})};

  /// \brief Last simulation time we update markers.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief Protects variables that are updated by the user.
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
VisualizePose::VisualizePose()
  : GuiSystem(), dataPtr(std::make_unique<VisualizePosePrivate>())
{
  qRegisterMetaType<Entity>("Entity");
}

/////////////////////////////////////////////////
VisualizePose::~VisualizePose()
{
  this->ClearPlot();
}

/////////////////////////////////////////////////
void VisualizePose::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Visualize Pose";

  // Parameters from SDF
  if (_pluginElem)
  {
    auto nameElem = _pluginElem->FirstChildElement("entity_name");
    if (nullptr != nameElem && nullptr != nameElem->GetText())
    {
      this->dataPtr->targetName = nameElem->GetText();
      this->dataPtr->targetNameDirty = true;
      this->SetLocked(true);
    }

    auto updateRateElem = _pluginElem->FirstChildElement("update_rate");
    if (nullptr != updateRateElem && nullptr != updateRateElem->GetText())
    {
      double updateRate = this->UpdateRate();
      updateRateElem->QueryDoubleText(&updateRate);
      this->SetUpdateRate(updateRate);
    }

    auto ptsElem = _pluginElem->FirstChildElement("maximum_points");
    if (nullptr != ptsElem && nullptr != ptsElem->GetText())
    {
      ptsElem->QueryIntText(&this->dataPtr->maxPoints);
      this->MaxPointsChanged();
    }
  }

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void VisualizePose::ClearPlot()
{
  // Clear all markers in current namespace
  this->dataPtr->markerMsg.set_action(msgs::Marker::DELETE_ALL);
  this->dataPtr->node.Request("/marker", this->dataPtr->markerMsg);
}

//////////////////////////////////////////////////
/// \note some marker types not supported by render engine:
/// CONE, ARROW, AXIS
///
/// gz-gui/src/plugins/marker_manager/MarkerManager.cc L634
/// gz-rendering/include/gz/rendering/MarkerManager.hh L34
///
void VisualizePose::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // Check elapsed time since last update.
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
  {
    return;
  }
  this->dataPtr->lastUpdateTime = _info.simTime;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  bool newTarget{false};

  // New target by name, get entity
  if (this->dataPtr->targetNameDirty)
  {
    auto entities = entitiesFromScopedName(this->dataPtr->targetName, _ecm);
    if (entities.empty())
    {
      // Keep trying
      return;
    }

    Entity entity = *(entities.begin());

    if (kNullEntity == entity)
    {
      // Keep trying
      return;
    }

    this->dataPtr->targetEntity = entity;
    this->dataPtr->targetNameDirty = false;
    newTarget = true;
  }

  // New target by entity, get name
  if (this->dataPtr->targetEntityDirty)
  {
    this->dataPtr->targetEntityDirty = false;

    auto name = _ecm.ComponentData<components::Name>(
        this->dataPtr->targetEntity);
    if (!name)
    {
      this->dataPtr->targetName.clear();
      return;
    }
    this->dataPtr->targetName = name.value();

    newTarget = true;
  }

  // Axis radius and length
  double radius = 0.02;
  double length = 0.2;

  if (newTarget || this->dataPtr->markerMsgs.marker().empty())
  {
    this->ClearPlot();

    auto ns = "pose_" + this->dataPtr->targetName;

    // Reset message
    this->dataPtr->markerMsg.Clear();
    this->dataPtr->markerMsg.set_ns(ns);
    this->dataPtr->markerMsg.set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg.set_visibility(msgs::Marker::GUI);

    this->dataPtr->markerMsgs.Clear();

    // x-axis
    auto transR = math::Color::Red;
    auto transG = math::Color::Green;
    auto transB = math::Color::Blue;
    auto transY = math::Color::Yellow;
    transR.A() = 0.7;
    transG.A() = 0.7;
    transB.A() = 0.7;
    transY.A() = 0.7;

    this->dataPtr->markerMsg1 = this->dataPtr->markerMsgs.add_marker();
    this->dataPtr->markerMsg1->set_ns(ns);
    this->dataPtr->markerMsg1->set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg1->set_type(msgs::Marker::CYLINDER);
    this->dataPtr->markerMsg1->set_visibility(msgs::Marker::GUI);
    msgs::Set(this->dataPtr->markerMsg1->mutable_scale(),
      math::Vector3d(radius, radius, length));
    msgs::Set(this->dataPtr->markerMsg1->mutable_material()->mutable_ambient(),
      transR);
    msgs::Set(this->dataPtr->markerMsg1->mutable_material()->mutable_diffuse(),
      transR);

    // y-axis
    this->dataPtr->markerMsg2 = this->dataPtr->markerMsgs.add_marker();
    this->dataPtr->markerMsg2->set_ns(ns);
    this->dataPtr->markerMsg2->set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg2->set_type(msgs::Marker::CYLINDER);
    this->dataPtr->markerMsg2->set_visibility(msgs::Marker::GUI);
    msgs::Set(this->dataPtr->markerMsg2->mutable_scale(),
      math::Vector3d(radius, radius, length));
    msgs::Set(this->dataPtr->markerMsg2->mutable_material()->mutable_ambient(),
      transG);
    msgs::Set(this->dataPtr->markerMsg2->mutable_material()->mutable_diffuse(),
      transG);

    // z-axis
    this->dataPtr->markerMsg3 = this->dataPtr->markerMsgs.add_marker();
    this->dataPtr->markerMsg3->set_ns(ns);
    this->dataPtr->markerMsg3->set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg3->set_type(msgs::Marker::CYLINDER);
    this->dataPtr->markerMsg3->set_visibility(msgs::Marker::GUI);
    msgs::Set(this->dataPtr->markerMsg3->mutable_scale(),
      math::Vector3d(radius, radius, length));
    msgs::Set(this->dataPtr->markerMsg3->mutable_material()->mutable_ambient(),
      transB);
    msgs::Set(this->dataPtr->markerMsg3->mutable_material()->mutable_diffuse(),
      transB);

    // origin
    this->dataPtr->markerMsg4 = this->dataPtr->markerMsgs.add_marker();
    this->dataPtr->markerMsg4->set_ns(ns);
    this->dataPtr->markerMsg4->set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg4->set_type(msgs::Marker::SPHERE);
    this->dataPtr->markerMsg4->set_visibility(msgs::Marker::GUI);
    msgs::Set(this->dataPtr->markerMsg4->mutable_scale(),
      math::Vector3d(radius, radius, radius));
    msgs::Set(this->dataPtr->markerMsg4->mutable_material()->mutable_ambient(),
      transY);
    msgs::Set(this->dataPtr->markerMsg4->mutable_material()->mutable_diffuse(),
      transY);

    // Update view
    this->TargetEntityChanged();
    this->TargetNameChanged();
  }

  // Clean up markers when maxPoints changes
  if (this->dataPtr->maxPointsDirty)
  {
    this->ClearPlot();
    this->dataPtr->maxPointsDirty = false;
  }

  // Nothing further to do if there are no poses to plot.
  if (this->dataPtr->targetEntity == kNullEntity ||
      this->dataPtr->maxPoints <= 0)
    return;

  // Get entity pose
  if (!_ecm.Component<components::WorldPose>(this->dataPtr->targetEntity))
  {
    _ecm.CreateComponent(this->dataPtr->targetEntity, components::WorldPose());
  }
  auto pose = worldPose(this->dataPtr->targetEntity, _ecm);

  // Translate and rotate each axis
  auto xPose = pose * math::Pose3d(length / 2, 0, 0, 0, GZ_PI / 2, 0);
  auto yPose = pose * math::Pose3d(0, length / 2, 0, -GZ_PI / 2, 0, 0);
  auto zPose = pose * math::Pose3d(0, 0, length / 2, 0, 0, 0);

  // Set markerID
  /// \note it appears that markerID = 0 is special for some reason, and does
  /// not get modified or deleted. Must start with markerID = 1.
  int markerID1 = this->dataPtr->markerID + 1;
  int markerID2 = markerID1 + this->dataPtr->maxPoints;
  int markerID3 = markerID2 + this->dataPtr->maxPoints;
  int markerID4 = markerID3 + this->dataPtr->maxPoints;
  this->dataPtr->markerMsg1->set_id(markerID1);
  this->dataPtr->markerMsg2->set_id(markerID2);
  this->dataPtr->markerMsg3->set_id(markerID3);
  this->dataPtr->markerMsg4->set_id(markerID4);

  // Set pose
  msgs::Set(this->dataPtr->markerMsg1->mutable_pose(), xPose);
  msgs::Set(this->dataPtr->markerMsg2->mutable_pose(), yPose);
  msgs::Set(this->dataPtr->markerMsg3->mutable_pose(), zPose);
  msgs::Set(this->dataPtr->markerMsg4->mutable_pose(), pose);

  // Increment the markerID
  this->dataPtr->markerID =
    (this->dataPtr->markerID + 1) % this->dataPtr->maxPoints;

  /// \todo(srmainwaring) The non-blocking version does not appears to work?
  /// Use a minimal timeout to prevent simulation stalling...

  // Publish the markers simultaneously.
  msgs::Boolean reply;
  bool result;
  unsigned int timeout = 1;
  this->dataPtr->node.Request("/marker_array", this->dataPtr->markerMsgs,
    timeout, reply, result);
  // this->dataPtr->node.Request("/marker_array", this->dataPtr->markerMsgs);
}

/////////////////////////////////////////////////
bool VisualizePose::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == sim::gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetTargetEntity(*event->Data().begin());
      }
    }

    if (_event->type() == sim::gui::events::DeselectAllEntities::kType)
    {
      auto event = reinterpret_cast<gui::events::DeselectAllEntities *>(
          _event);
      if (event)
      {
        this->SetTargetEntity(kNullEntity);
      }
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
Entity VisualizePose::TargetEntity() const
{
  return this->dataPtr->targetEntity;
}

/////////////////////////////////////////////////
void VisualizePose::SetTargetEntity(Entity _entity)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->targetEntity = _entity;
  this->dataPtr->targetEntityDirty = true;
  this->TargetEntityChanged();

  if (this->dataPtr->targetEntity == kNullEntity)
  {
    this->dataPtr->targetName.clear();
  }
}

/////////////////////////////////////////////////
QString VisualizePose::TargetName() const
{
  return QString::fromStdString(this->dataPtr->targetName);
}

/////////////////////////////////////////////////
void VisualizePose::SetTargetName(const QString &_targetName)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->targetName = _targetName.toStdString();
  this->dataPtr->targetNameDirty = true;
  this->TargetNameChanged();
}

/////////////////////////////////////////////////
bool VisualizePose::Locked() const
{
  return this->dataPtr->locked;
}

/////////////////////////////////////////////////
void VisualizePose::SetLocked(bool _locked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->locked = _locked;
  this->LockedChanged();
}

/////////////////////////////////////////////////
double VisualizePose::UpdateRate() const
{
  double period = std::chrono::duration<double>(
    this->dataPtr->updatePeriod).count();
  double rate = 1.0/period;

  gzdbg << "UpdateRate:"
        << " rate: " << rate << " Hz"
        << ", period: " << period << " s"
        << "\n";

  return rate;
}

/////////////////////////////////////////////////
void VisualizePose::SetUpdateRate(double _updateRate)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update period in seconds
  std::chrono::duration<double> period{1.0 / _updateRate};
  this->dataPtr->updatePeriod =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  gzdbg << "SetUpdateRate:"
        << " rate: " << _updateRate << " Hz"
        << ", period: " << period.count() << " s"
        << "\n";

  this->UpdateRateChanged();
}

/////////////////////////////////////////////////
int VisualizePose::MaxPoints() const
{
  gzdbg << "MaxPoints:"
        << " maxPoints: " << this->dataPtr->maxPoints
        << "\n";

  return this->dataPtr->maxPoints;
}

/////////////////////////////////////////////////
void VisualizePose::SetMaxPoints(int _maxPoints)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->maxPoints = _maxPoints;
  this->dataPtr->maxPointsDirty = true;

  gzdbg << "SetMaxPoints:"
        << " maxPoints: " << this->dataPtr->maxPoints
        << "\n";

  this->MaxPointsChanged();
}

}  // namespace gui
}  // namespace sim
}  // namespace gz

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::gui::VisualizePose,
              gz::gui::Plugin)
