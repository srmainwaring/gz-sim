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

#ifndef GZ_SIM_GUI_VISUALIZEPOSE_HH_
#define GZ_SIM_GUI_VISUALIZEPOSE_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
namespace gui
{

class VisualizePosePrivate;

/// \brief Visualise the pose of an entity in the 3D scene.
///
/// This plugin can be instantiated multiple times to display the pose of
/// various entities.
///
/// The plugin is automatically attached to the currently selected entity,
/// unless it is locked on an entity.
///
/// ## Configuration
///
/// * `<entity_name>` (optional): Plot the given entity at startup. Accepts
/// names scoped with `::`, for example `my_model::my_link`. If not provided,
/// the plugin starts not attached to any entity, and will attach to the
/// next selected entity.
///
/// * `<update_rate>` (optional): The update rate at which to display
///  the pose. Defaults to 1 Hz.
///
/// * `<maximum_points> (optional)`: Maximum number of poses to display.
/// After this number is reached, the older poses start being deleted.
/// Defaults to 50.
///
class VisualizePose : public gz::sim::GuiSystem
{
  Q_OBJECT

  /// \brief Target entity
  Q_PROPERTY(
    Entity targetEntity
    READ TargetEntity
    WRITE SetTargetEntity
    NOTIFY TargetEntityChanged
  )

  /// \brief Target entity scoped name
  Q_PROPERTY(
    QString targetName
    READ TargetName
    WRITE SetTargetName
    NOTIFY TargetNameChanged
  )

  /// \brief Whether the plugin is locked on an entity
  Q_PROPERTY(
    bool locked
    READ Locked
    WRITE SetLocked
    NOTIFY LockedChanged
  )

  /// \brief Update rate (Hz)
  Q_PROPERTY(
    double updateRate
    READ UpdateRate
    WRITE SetUpdateRate
    NOTIFY UpdateRateChanged
  )

  /// \brief Maximum number of total points on the plot
  Q_PROPERTY(
    int maxPoints
    READ MaxPoints
    WRITE SetMaxPoints
    NOTIFY MaxPointsChanged
  )

  /// \brief Constructor
  public: VisualizePose();

  /// \brief Destructor
  public: ~VisualizePose() override;

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  /// \brief Get the target currently controlled.
  /// \return Target entity ID.
  public: Q_INVOKABLE Entity TargetEntity() const;

  /// \brief Set the target currently controlled.
  /// \param[in] _entity Target entity ID.
  public: Q_INVOKABLE void SetTargetEntity(Entity _entity);

  /// \brief Notify that entity has changed.
  signals: void TargetEntityChanged();

  /// \brief Get the name of target currently controlled.
  /// \return TargetName, such as 'world' or 'target'
  public: Q_INVOKABLE QString TargetName() const;

  /// \brief Set the name of target currently controlled.
  /// \param[in] _name TargetName, such as 'world' or 'target'.
  public: Q_INVOKABLE void SetTargetName(const QString &_name);

  /// \brief Notify that target name has changed
  signals: void TargetNameChanged();

  /// \brief Get whether the plugin is currently locked on a target.
  /// \return True for locked
  public: Q_INVOKABLE bool Locked() const;

  /// \brief Set whether the plugin is currently locked on a target.
  /// \param[in] _locked True for locked.
  public: Q_INVOKABLE void SetLocked(bool _locked);

  /// \brief Notify that locked has changed.
  signals: void LockedChanged();

  /// \brief Get the update rate.
  /// \return The current update rate.
  public: Q_INVOKABLE double UpdateRate() const;

  /// \brief Set the update rate.
  /// \param[in] _updateRate New update rate.
  public: Q_INVOKABLE void SetUpdateRate(double _updateRate);

  /// \brief Notify that the update rate has changed.
  signals: void UpdateRateChanged();

  /// \brief Get the maximum number of points.
  /// \return The current maximum points.
  public: Q_INVOKABLE int MaxPoints() const;

  /// \brief Set the maximum number of points. If the plot has more than
  /// this number, older points start being removed.
  /// \param[in] _maxPoints Maximum number of points.
  public: Q_INVOKABLE void SetMaxPoints(int _maxPoints);

  /// \brief Notify that the maximum points has changed.
  signals: void MaxPointsChanged();

  // Documentation inherited
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief Clear plot
  private: void ClearPlot();

  /// \internal
  /// \brief Pointer to private data
  private: std::unique_ptr<VisualizePosePrivate> dataPtr;
};

}  // namespace gui
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_GUI_VISUALIZEPOSE_HH_
