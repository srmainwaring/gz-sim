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

#include "ForceDisplay.hh"

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

/// \brief Private data class for ForceDisplay
class ForceDisplayPrivate
{
  /// \brief Perform render operations - runs on render thread.
  public: void PerformRenderingOperations();

  /// \brief Find the current render engine and scene.
  public: void FindScene();

  /// \brief Clear display
  public: void ClearDisplay();

  // public: void Subscribe();

  // public: void Unsubscribe();

  public: void OnPose(const msgs::Pose &_msg);

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief The wrench topic.
  public: std::string topic = "/wrench";

  /// \brief Maximum number of wrenches to display. Older wrenches are deleted.
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

  public: bool showForces{true};
  public: math::Color color{math::Color::Red};
  public: double shaftLength{0.5};
  public: double shaftRadius{0.05};
  public: double headLength{0.25};
  public: double headRadius{0.1};



  // Render operations
  public: rendering::ScenePtr scene;
  public: bool sceneDirty{true};

  // Store wrench history - filter size
  public: std::vector<rendering::ArrowVisualPtr> arrowVisuals;

  // Pose index modulo filter size
  public: int wrenchIndex{0};

  /// \brief Last message received.
  public: msgs::Pose poseMsg;

  /// \brief Protects variables that are updated by the user.
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
void ForceDisplayPrivate::PerformRenderingOperations()
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
    this->wrenchIndex = 0;
    this->filterSizeDirty = false;
  }

  int idx = this->wrenchIndex;

  auto updateArrow = [this](rendering::ArrowVisualPtr _visual)
  {
    // update material
    rendering::MaterialPtr mat = this->scene->CreateMaterial();
    mat->SetAmbient(this->color);
    mat->SetDiffuse(this->color);
    mat->SetSpecular(0.5, 0.5, 0.5, this->color.A());
    mat->SetShininess(50);
    mat->SetReflectivity(0);
    mat->SetCastShadows(false);
    _visual->SetMaterial(mat);
    this->scene->DestroyMaterial(mat);

    // update dimensions
    _visual->Shaft()->SetOrigin(0.0, 0.0, 0.5);
    _visual->Shaft()->SetLocalScale(
      this->shaftRadius, this->shaftRadius, this->shaftLength);

    _visual->Head()->SetOrigin(0.0, 0.0, -0.5);
    _visual->Head()->SetLocalScale(
      this->headRadius, this->headRadius, this->headLength);

    _visual->SetVisible(this->showForces);
  };

  // Create arrow visuals
  if (this->arrowVisuals[idx] == nullptr)
  {
    rendering::MaterialPtr mat = this->scene->CreateMaterial();    
    mat->SetAmbient(this->color);
    mat->SetDiffuse(this->color);
    mat->SetSpecular(0.5, 0.5, 0.5, this->color.A());
    mat->SetShininess(50);
    mat->SetReflectivity(0);
    mat->SetCastShadows(false);

    auto visual = this->scene->CreateArrowVisual();
    updateArrow(visual);

    visual->ShowArrowHead(true);
    visual->ShowArrowShaft(true);
    visual->ShowArrowRotation(false);
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    visual->SetVisible(this->showForces);

    this->arrowVisuals[idx] = visual;
    rootVisual->AddChild(this->arrowVisuals[idx]);

    // gzdbg << "Add Arrow[" << idx << "], Id["
    //       << visual->Id() << "]\n";
  }

  // switch shape
  if (this->shapeDirty)
  {
    for (auto&& visual : this->arrowVisuals)
    {
      if (visual != nullptr)
      {
        updateArrow(visual);
        visual->SetVisible(this->showForces);
      }
    }

    this->shapeDirty = false;
  }

  // Set pose
  math::Pose3d pose = msgs::Convert(this->poseMsg);
  this->arrowVisuals[idx]->SetWorldPose(
      pose * math::Pose3d(0, 0, 0, 0, GZ_PI/2, 0));

  // gzdbg << "Render:\n"
  //       << " Node count [" << this->scene->NodeCount() << "].\n"
  //       << " Visual count [" << this->scene->VisualCount() << "].\n";

  this->wrenchIndex = (this->wrenchIndex + 1) % this->filterSize;

  this->sceneDirty = false;
}

/////////////////////////////////////////////////
void ForceDisplayPrivate::FindScene()
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
void ForceDisplayPrivate::ClearDisplay()
{
  // gzdbg << "Filter size ["
  //       << this->filterSize
  //       << "].\n"
  //       << "Removing ["
  //       << this->arrowVisuals.size()
  //       << "] arrows.\n";

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

  // gzdbg << "After Clear\n"
  //       << "Node count [" << this->scene->NodeCount() << "].\n"
  //       << "Visual count [" << this->scene->VisualCount() << "].\n";
}

/////////////////////////////////////////////////
void ForceDisplayPrivate::OnPose(const msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->poseMsg = _msg;
  this->sceneDirty = true;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
ForceDisplay::ForceDisplay()
  : gz::gui::Plugin(), dataPtr(std::make_unique<ForceDisplayPrivate>())
{
}

/////////////////////////////////////////////////
ForceDisplay::~ForceDisplay()
{
}

/////////////////////////////////////////////////
void ForceDisplay::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
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
      auto elem = _pluginElem->FirstChildElement("color");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        std::stringstream ss(elem->GetText());
        math::Color color;
        ss >> color; 
        this->SetColor(gz::gui::convert(color));
      }
    }
  }

  // Install filter to receive events from the main window.
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool ForceDisplay::eventFilter(QObject *_obj, QEvent *_event)
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
QString ForceDisplay::Topic() const
{
  gzdbg << "Topic: " << this->dataPtr->topic << "\n";

  return QString::fromStdString(this->dataPtr->topic);
}

/////////////////////////////////////////////////
void ForceDisplay::SetTopic(const QString& _topic)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  std::string newTopic = _topic.toStdString();

  // Unscribe to current topic
  this->dataPtr->node.Unsubscribe(this->dataPtr->topic);

  // Update subscription
  this->dataPtr->node.Subscribe(newTopic,
      &ForceDisplayPrivate::OnPose, this->dataPtr.get());

  // Store topic and flag dirty.
  this->dataPtr->topic = newTopic;
  this->dataPtr->sceneDirty = true;
  this->TopicChanged();

  gzdbg << "SetTopic: " << this->dataPtr->topic << "\n";
}

/////////////////////////////////////////////////
double ForceDisplay::UpdateRate() const
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
void ForceDisplay::SetUpdateRate(double _updateRate)
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
QColor ForceDisplay::Color() const
{
  gzdbg << "Color: " << this->dataPtr->color << "\n";

  return gz::gui::convert(this->dataPtr->color);
}

/////////////////////////////////////////////////
void ForceDisplay::SetColor(const QColor &_color)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->color = gz::gui::convert(_color);
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->ColorChanged();

  gzdbg << "SetColor: " << this->dataPtr->color << "\n";
}

/////////////////////////////////////////////////
double ForceDisplay::Alpha() const
{
  gzdbg << "Alpha: " << this->dataPtr->color.A() << "\n";

  return this->dataPtr->color.A();
}

/////////////////////////////////////////////////
void ForceDisplay::SetAlpha(double _alpha)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->color.A(_alpha);
  this->dataPtr->shapeDirty = true;
  this->dataPtr->sceneDirty = true;
  this->AlphaChanged();

  gzdbg << "Alpha: " << this->dataPtr->color.A() << "\n";
}

}  // namespace gui
}  // namespace sim
}  // namespace gz

/////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::gui::ForceDisplay,
              gz::gui::Plugin)
