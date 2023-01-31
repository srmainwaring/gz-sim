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

#include "PoseDisplay.hh"

#include <algorithm>
#include <mutex>
#include <string>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs.hh>
#include <gz/msgs/Utility.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/ArrowVisual.hh>
#include <gz/rendering/AxisVisual.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/Util.hh"

#include <gz/transport/Node.hh>

namespace gz
{
namespace rendering
{
void DebugNodeRecursive(NodePtr _node,
    std::set<unsigned int> &_nodeIds,
    int level = 0)
{
  gzdbg << "Level[" << level << "] "
        << "Node[" << _node->Id() << "] "
        << "Parent[" << _node->Parent()->Id() << "] "
        << "ChildCount[" << _node->ChildCount() << "]\n";

  // check if we have visited this node before
  if (_nodeIds.find(_node->Id()) != _nodeIds.end())
  {
    gzwarn << "Detected loop in scene tree.\n";
    return;
  }
  _nodeIds.insert(_node->Id());

  // debug child nodes first
  for (auto i = 0; i < _node->ChildCount(); ++i)
  {
    DebugNodeRecursive(_node->ChildByIndex(i), _nodeIds, level + 1);
  }
}

void DebugNode(NodePtr _node)
{
  std::set<unsigned int> nodeIds;
  DebugNodeRecursive(_node, nodeIds);
}
}  // namespace rendering

namespace sim
{
namespace gui
{

/// \brief Private data class for PoseDisplay
class PoseDisplayPrivate
{
  /// \brief Perform render operations - runs on render thread.
  public: void PerformRenderingOperations();

  /// \brief Find the current render engine and scene.
  public: void FindScene();

  /// \brief Clear pose display
  public: void ClearDisplay();

  // public: void Subscribe();

  // public: void Unsubscribe();

  public: void OnPose(const msgs::Pose &_msg);

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief The pose topic.
  public: std::string topic = "/pose";

  /// \brief Maximum number of poses to display. Older poses are deleted.
  public: int filterSize{50};

  /// \brief Whether the max points has been changed.
  public: bool filterSizeDirty{true};

  /// \brief Time between marker updates (secs).
  public: std::chrono::steady_clock::duration updatePeriod{
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::seconds{1})};

  /// \brief Last simulation time we update markers.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  // Properties
  public: bool shapeDirty{true};

  public: int shapeIndex{0};
  public: math::Color color{math::Color::Red};
  public: double shaftLength{1.0};
  public: double shaftRadius{0.05};
  public: double headLength{0.3};
  public: double headRadius{0.1};
  public: double axesLength{1.0};
  public: double axesRadius{0.1};

  // Render operations
  public: rendering::ScenePtr scene;
  public: bool sceneDirty{true};

  // Store pose history - filter size
  public: std::vector<rendering::ArrowVisualPtr> arrowVisuals;
  public: std::vector<rendering::AxisVisualPtr> axisVisuals;

  // Pose index modulo filter size
  public: int poseIndex{0};

  /// \brief Last pose message received.
  public: msgs::Pose poseMsg;

  /// \brief Protects variables that are updated by the user.
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
void PoseDisplayPrivate::PerformRenderingOperations()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!this->sceneDirty)
  {
    return;
  }

  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    return;

  // Get root visual (it is checked valid in FindScene)
  auto rootVisual = this->scene->RootVisual();

  /// \todo manage update without clearing down history...
  // Resize visual arrays
  if (this->filterSizeDirty)
  {
    this->ClearDisplay();
    this->arrowVisuals.resize(this->filterSize);
    this->axisVisuals.resize(this->filterSize);
    this->poseIndex = 0;
    this->filterSizeDirty = false;
  }

  int i = this->poseIndex;

  // Create arrow visuals
  if (this->arrowVisuals[i] == nullptr)
  {
    rendering::MaterialPtr mat = this->scene->CreateMaterial();    
    mat->SetAmbient(this->color);
    mat->SetDiffuse(this->color);
    mat->SetSpecular(0.5, 0.5, 0.5, this->color.A());
    mat->SetShininess(50);
    mat->SetReflectivity(0);
    mat->SetCastShadows(false);

    auto visual = this->scene->CreateArrowVisual();
    visual->SetMaterial(mat);
    this->scene->DestroyMaterial(mat);

    visual->SetLocalScale(0.5, 0.5, 0.5);
    visual->ShowArrowHead(true);
    visual->ShowArrowShaft(true);
    visual->ShowArrowRotation(false);
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    visual->SetVisible(this->shapeIndex == 0);

    this->arrowVisuals[i] = visual;
    rootVisual->AddChild(this->arrowVisuals[i]);

    // gzdbg << "Add Arrow[" << i << "], Id["
    //       << visual->Id() << "]\n";
  }

  // Create axis visuals
  if (this->axisVisuals[i] == nullptr)
  {
    auto visual = this->scene->CreateAxisVisual();
    visual->ShowAxisHead(false);
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    visual->SetVisible(this->shapeIndex == 1);

    this->axisVisuals[i] = visual;
    rootVisual->AddChild(this->axisVisuals[i]);

    // gzdbg << "Add Axis[" << i << "], Id["
    //       << visual->Id() << "]\n";
  }

  // switch shape
  if (this->shapeDirty)
  {
    for (auto i = 0; i < this->filterSize; ++i)
    {
      if (this->arrowVisuals[i] != nullptr)
      {
        // update material
        rendering::MaterialPtr mat = this->scene->CreateMaterial();    
        mat->SetAmbient(this->color);
        mat->SetDiffuse(this->color);
        mat->SetSpecular(0.5, 0.5, 0.5, this->color.A());
        mat->SetShininess(50);
        mat->SetReflectivity(0);
        mat->SetCastShadows(false);
        this->arrowVisuals[i]->SetMaterial(mat);
        this->scene->DestroyMaterial(mat);

        // update dimensions
        this->arrowVisuals[i]->Shaft()->SetLocalScale(
          this->shaftRadius * 2.0,
          this->shaftRadius * 2.0,
          this->shaftLength);
        this->arrowVisuals[i]->Shaft()->SetOrigin(
          0.0,
          0.0,
          this->shaftLength * -1.0);
        this->arrowVisuals[i]->Head()->SetLocalScale(
          this->headRadius * 2.0,
          this->headRadius * 2.0,
          this->headLength * 2.0);

        this->arrowVisuals[i]->SetVisible(this->shapeIndex == 0);
      }

      if (this->axisVisuals[i] != nullptr)
      {
        // update dimensions
        for (int j = 0; j < 3; ++j)
        {
          auto visual = this->axisVisuals[i]->ChildByIndex(j);
          visual->SetLocalScale(
            this->axesRadius * 20.0,
            this->axesRadius * 20.0,
            this->axesLength * 2.0);
        }

        this->axisVisuals[i]->SetVisible(this->shapeIndex == 1);
      }
    }
    this->shapeDirty = false;
  }

  // Set pose
  math::Pose3d pose = msgs::Convert(this->poseMsg);
  this->arrowVisuals[i]->SetWorldPose(
      pose * math::Pose3d(0, 0, 0, 0, GZ_PI/2, 0));
  this->axisVisuals[i]->SetWorldPose(pose);

  // gzdbg << "Render:\n"
  //       << " Node count [" << this->scene->NodeCount() << "].\n"
  //       << " Visual count [" << this->scene->VisualCount() << "].\n";

  this->poseIndex = (this->poseIndex + 1) % this->filterSize;

  this->sceneDirty = false;
}

/////////////////////////////////////////////////
void PoseDisplayPrivate::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scenePtr->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}

/////////////////////////////////////////////////
void PoseDisplayPrivate::ClearDisplay()
{
  // gzdbg << "Filter size ["
  //       << this->filterSize
  //       << "].\n"
  //       << "Removing ["
  //       << this->arrowVisuals.size()
  //       << "] arrows.\n"
  //       << "Removing ["
  //       << this->axisVisualsVisuals.size()
  //       << "] axis.\n";

  // gzdbg << "Before Clear\n"
  //       << "Node count [" << this->scene->NodeCount() << "].\n"
  //       << "Visual count [" << this->scene->VisualCount() << "].\n";

  int i = 0;
  for (auto&& visual : this->arrowVisuals)
  {
    if (visual != nullptr && visual->HasParent())
    {
      // gzdbg << "Removing Arrow[" << i++ << "], Id["
      //       << visual->Id() << "] \n";
      visual->Parent()->RemoveChild(visual);
      this->scene->DestroyVisual(visual, true);
      visual.reset();
    }
  }
  this->arrowVisuals.clear();

  i = 0;
  for (auto&& visual : this->axisVisuals)
  {
    if (visual != nullptr && visual->HasParent())
    {
      // gzdbg << "Removing Axis[" << i++ << "], Id["
      //       << visual->Id() << "] \n";
      visual->Parent()->RemoveChild(visual);
      this->scene->DestroyVisual(visual, true);
      visual.reset();
    }
  }
  this->axisVisuals.clear();

  // gzdbg << "After Clear\n"
  //       << "Node count [" << this->scene->NodeCount() << "].\n"
  //       << "Visual count [" << this->scene->VisualCount() << "].\n";
}

/////////////////////////////////////////////////
/// \todo requires recursive mutex
// void PoseDisplayPrivate::Subscribe()
// {
//   std::lock_guard<std::mutex> lock(this->mutex);
//   this->dataPtr->node.Subscribe(newTopic,
//       &PoseDisplayPrivate::OnPose, this->dataPtr.get());
// }

/////////////////////////////////////////////////
// void PoseDisplayPrivate::Unsubscribe()
// {
//   std::lock_guard<std::mutex> lock(this->mutex);
//   this->dataPtr->node.Unsubscribe(this->dataPtr->topic);
// }

/////////////////////////////////////////////////
void PoseDisplayPrivate::OnPose(const msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->poseMsg = _msg;
  this->sceneDirty = true;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
PoseDisplay::PoseDisplay()
  : gz::gui::Plugin(), dataPtr(std::make_unique<PoseDisplayPrivate>())
{
}

/////////////////////////////////////////////////
PoseDisplay::~PoseDisplay()
{
}

/////////////////////////////////////////////////
void PoseDisplay::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Pose Display";

  /// \todo(srmainwaring) move subscription into separate function 
  // Initialise subscription
  this->SetTopic(QString::fromStdString(this->dataPtr->topic));

  // Parameters from SDF
  if (_pluginElem)
  {
    {
      auto elem = _pluginElem->FirstChildElement("topic");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        std::string topic = elem->GetText();
        this->SetTopic(QString::fromStdString(topic));
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("update_rate");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double updateRate(0.0);
        elem->QueryDoubleText(&updateRate);
        this->SetUpdateRate(updateRate);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("shape");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        std::string shape = elem->GetText();
        std::transform(shape.begin(), shape.end(), shape.begin(), ::toupper);
        int shapeIndex = this->dataPtr->shapeIndex;
        if (shape == "ARROW")
          shapeIndex = 0;
        else if (shape == "AXIS")
          shapeIndex = 1;
        else
        {
          gzwarn << "Invalid shape [" << shape << "]. "
                 << "Must be 'ARROW' or 'AXIS'.\n";
        }
        this->SetShapeIndex(0);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("color");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        std::stringstream ss(elem->GetText());
        math::Color color;
        ss >> color; 
        this->SetColor(gz::gui::convert(color));
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("shaft_length");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double shaftLength{0.0};
        elem->QueryDoubleText(&shaftLength);
        this->SetShaftLength(shaftLength);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("shaft_radius");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double shaftRadius{0.0};
        elem->QueryDoubleText(&shaftRadius);
        this->SetShaftRadius(shaftRadius);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("head_length");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double headLength{0.0};
        elem->QueryDoubleText(&headLength);
        this->SetHeadLength(headLength);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("head_radius");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double headRadius{0.0};
        elem->QueryDoubleText(&headRadius);
        this->SetHeadRadius(headRadius);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("axes_length");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double axesLength{0.0};
        elem->QueryDoubleText(&axesLength);
        this->SetAxesLength(axesLength);
      }
    }

    {
      auto elem = _pluginElem->FirstChildElement("axes_radius");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double axisRadius{0.0};
        elem->QueryDoubleText(&axisRadius);
        this->SetAxesRadius(axisRadius);
      }
    }
  }

  // Install filter to receive events from the main window.
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool PoseDisplay::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the render thread, so it's safe to make
    // rendering calls here
    this->dataPtr->PerformRenderingOperations();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
QString PoseDisplay::Topic() const
{
  gzdbg << "Topic: " << this->dataPtr->topic << "\n";

  return QString::fromStdString(this->dataPtr->topic);
}

/////////////////////////////////////////////////
void PoseDisplay::SetTopic(const QString& _topic)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  std::string newTopic = _topic.toStdString();

  // Unscribe to current topic
  this->dataPtr->node.Unsubscribe(this->dataPtr->topic);

  // Update subscription
  this->dataPtr->node.Subscribe(newTopic,
      &PoseDisplayPrivate::OnPose, this->dataPtr.get());

  // Store topic and flag dirty.
  this->dataPtr->topic = newTopic;
  this->dataPtr->sceneDirty = true;
  this->TopicChanged();

  gzdbg << "SetTopic: " << this->dataPtr->topic << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::UpdateRate() const
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
void PoseDisplay::SetUpdateRate(double _updateRate)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update period in seconds
  std::chrono::duration<double> period{1.0 / _updateRate};
  this->dataPtr->updatePeriod =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  this->dataPtr->sceneDirty = true;
  this->UpdateRateChanged();

  gzdbg << "SetUpdateRate:"
        << " rate: " << _updateRate << " Hz"
        << ", period: " << period.count() << " s"
        << "\n";
}

/////////////////////////////////////////////////
int PoseDisplay::FilterSize() const
{
  gzdbg << "FilterSize: " << this->dataPtr->filterSize << "\n";

  return this->dataPtr->filterSize;
}

/////////////////////////////////////////////////
void PoseDisplay::SetFilterSize(int _filterSize)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->filterSize = _filterSize;
  this->dataPtr->filterSizeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->FilterSizeChanged();

  gzdbg << "SetFilterSize: " << this->dataPtr->filterSize << "\n";
}

/////////////////////////////////////////////////
int PoseDisplay::ShapeIndex() const
{
  gzdbg << "ShapeIndex: " << this->dataPtr->shapeIndex << "\n";

  return this->dataPtr->shapeIndex;
}

/////////////////////////////////////////////////
void PoseDisplay::SetShapeIndex(int _shapeIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->shapeIndex = _shapeIndex;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->ShapeIndexChanged();

  gzdbg << "SetShapeIndex: " << this->dataPtr->shapeIndex << "\n";
}

/////////////////////////////////////////////////
QColor PoseDisplay::Color() const
{
  gzdbg << "Color: " << this->dataPtr->color << "\n";

  return gz::gui::convert(this->dataPtr->color);
}

/////////////////////////////////////////////////
void PoseDisplay::SetColor(const QColor &_color)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->color = gz::gui::convert(_color);
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->ColorChanged();

  gzdbg << "SetColor: " << this->dataPtr->color << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::Alpha() const
{
  gzdbg << "Alpha: " << this->dataPtr->color.A() << "\n";

  return this->dataPtr->color.A();
}

/////////////////////////////////////////////////
void PoseDisplay::SetAlpha(double _alpha)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->color.A(_alpha);
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->AlphaChanged();

  gzdbg << "Alpha: " << this->dataPtr->color.A() << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::ShaftLength() const
{
  gzdbg << "ShaftLength: " << this->dataPtr->shaftLength << "\n";

  return this->dataPtr->shaftLength;
}

/////////////////////////////////////////////////
void PoseDisplay::SetShaftLength(double _shaftLength)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->shaftLength = _shaftLength;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->ShaftLengthChanged();

  gzdbg << "ShaftLength: " << this->dataPtr->shaftLength << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::ShaftRadius() const
{
  gzdbg << "ShaftRadius: " << this->dataPtr->shaftRadius << "\n";

  return this->dataPtr->shaftRadius;
}

/////////////////////////////////////////////////
void PoseDisplay::SetShaftRadius(double _shaftRadius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->shaftRadius = _shaftRadius;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->ShaftRadiusChanged();

  gzdbg << "ShaftRadius: " << this->dataPtr->shaftRadius << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::HeadLength() const
{
  gzdbg << "HeadLength: " << this->dataPtr->headLength << "\n";

  return this->dataPtr->headLength;
}

/////////////////////////////////////////////////
void PoseDisplay::SetHeadLength(double _headLength)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->headLength = _headLength;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->HeadLengthChanged();

  gzdbg << "HeadLength: " << this->dataPtr->headLength << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::HeadRadius() const
{
  gzdbg << "HeadRadius: " << this->dataPtr->headRadius << "\n";

  return this->dataPtr->headRadius;
}

/////////////////////////////////////////////////
void PoseDisplay::SetHeadRadius(double _headRadius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->headRadius = _headRadius;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->HeadRadiusChanged();

  gzdbg << "HeadRadius: " << this->dataPtr->headRadius << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::AxesLength() const
{
  gzdbg << "AxesLength: " << this->dataPtr->axesLength << "\n";

  return this->dataPtr->axesLength;
}

/////////////////////////////////////////////////
void PoseDisplay::SetAxesLength(double _axesLength)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->axesLength = _axesLength;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->AxesLengthChanged();

  gzdbg << "AxesLength: " << this->dataPtr->axesLength << "\n";
}

/////////////////////////////////////////////////
double PoseDisplay::AxesRadius() const
{
  gzdbg << "AxesRadius: " << this->dataPtr->axesRadius << "\n";

  return this->dataPtr->axesRadius;
}

/////////////////////////////////////////////////
void PoseDisplay::SetAxesRadius(double _axesRadius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->axesRadius = _axesRadius;
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->AxesRadiusChanged();

  gzdbg << "AxesRadius: " << this->dataPtr->axesRadius << "\n";
}

}  // namespace gui
}  // namespace sim
}  // namespace gz

/////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::gui::PoseDisplay,
              gz::gui::Plugin)
