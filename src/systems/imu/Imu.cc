/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Imu.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/ImuSensor.hh>


#include "gz/sim/World.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Imu.hh"
#include <gz/sim/components/Inertial.hh>
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include "gz/sim/Util.hh"
#include <gz/sim/World.hh>

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Imu data class.
class gz::sim::systems::ImuPrivate
{
  /// \brief A map of IMU entity to its IMU sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::ImuSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldEntity = kNullEntity;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create IMU sensors in gz-sensors
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update IMU sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _imu IMU component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Imu *_imu,
    const components::ParentEntity *_parent);

  /// \brief Remove IMU sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveImuEntities(const EntityComponentManager &_ecm);

  // Custom Gravity

  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Model providing the source of gravity.
  public: Model sourceModel{kNullEntity};

  /// \brief Name of the gravity source model entity.
  public: std::string sourceName;

  /// \brief Power law exponent. Default is inverse square.
  public: double gravity_expo{-2.0};

  /// \brief Universal gravitational constant (N m^2 / kg^2).
  public: static constexpr double G{6.67430E-11};

};

//////////////////////////////////////////////////
Imu::Imu() : System(), dataPtr(std::make_unique<ImuPrivate>())
{
}

//////////////////////////////////////////////////
Imu::~Imu() = default;

//////////////////////////////////////////////////
void Imu::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Imu::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void Imu::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Imu::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the ImuPrivate::Update function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveImuEntities(_ecm);
}

//////////////////////////////////////////////////
void Imu::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &)
{
  // parameters
  if (_sdf->HasElement("gravity_source"))
  {
    this->dataPtr->sourceName = _sdf->Get<std::string>("gravity_source");
  }

  if (_sdf->HasElement("gravity_expo"))
  {
    this->dataPtr->gravity_expo = _sdf->Get<double>("gravity_expo");
  }
}

//////////////////////////////////////////////////
void ImuPrivate::AddSensor(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Imu *_imu,
  const components::ParentEntity *_parent)
{
  // Get the world acceleration (defined in world frame)
  auto gravity = _ecm.Component<components::Gravity>(worldEntity);
  if (nullptr == gravity)
  {
    gzerr << "World missing gravity." << std::endl;
    return;
  }

  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _imu->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/imu";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::ImuSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::ImuSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // set gravity - assume it remains fixed
  sensor->SetGravity(gravity->Data());

  // Get initial pose of sensor and set the reference z pos
  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
  math::Pose3d p = worldPose(_entity, _ecm);
  sensor->SetOrientationReference(p.Rot());

  // Get world frame orientation and heading.
  // If <orientation_reference_frame> includes a named
  // frame like NED, that must be supplied to the IMU sensor,
  // otherwise orientations are reported w.r.t to the initial
  // orientation.
  if (data.Element()->HasElement("imu")) {
    auto imuElementPtr = data.Element()->GetElement("imu");
    if (imuElementPtr->HasElement("orientation_reference_frame")) {
      double heading = 0.0;

      gz::sim::World world(worldEntity);
      if (world.SphericalCoordinates(_ecm))
      {
        auto sphericalCoordinates = world.SphericalCoordinates(_ecm).value();
        heading = sphericalCoordinates.HeadingOffset().Radian();
      }

      sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, heading),
        gz::sensors::WorldFrameEnumType::ENU);
    }
  }

  // Set whether orientation is enabled
  if (data.ImuSensor())
  {
    sensor->SetOrientationEnabled(
        data.ImuSensor()->OrientationEnabled());
  }

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void ImuPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ImuPrivate::CreateImuEntities");
  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  if (!this->initialized)
  {
    // resolve gravity source model
    if (!this->sourceName.empty())
    {
      this->world = World(this->worldEntity);

      this->sourceModel = Model(
          this->world.ModelByName(_ecm, this->sourceName));
      static bool notified{false};
      if (!this->sourceModel.Valid(_ecm) && !notified)
      {
        gzerr << "Imu: gravity source model ["
                << this->sourceName
                << "] not found. "
                "Failed to initialize.\n";
        notified = true;
        return;
      }
    }

    // Create IMUs
    _ecm.Each<components::Imu, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Imu *_imu,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _imu, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create IMUs
    _ecm.EachNew<components::Imu, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Imu *_imu,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _imu, _parent);
          return true;
      });
  }
}

//////////////////////////////////////////////////
void ImuPrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ImuPrivate::Update");

  // Custom gravity
  double mass_C = 0.0;
  math::Pose3d X_WC {};
  if (this->sourceModel.Valid(_ecm))
  {
    // canonical link of gravity source (C for central force)
    auto link_C = Link(this->sourceModel.CanonicalLink(_ecm));

    // world pose of source
    X_WC = worldPose(link_C.Entity(), _ecm);

    // mass of source
    auto *inertialComp_C =
      _ecm.Component<components::Inertial>(link_C.Entity());
    {
      static bool notified{false};
      if (inertialComp_C == nullptr)
      {
        gzerr << "Imu: gravity source model ["
                << this->sourceName
                << "] does not have valid inertial. "
                "Failed to initialize.\n";
        notified = true;
        return;
      }
    }
    const auto &inertial_C = inertialComp_C->Data();
    mass_C = inertial_C.MassMatrix().Mass();
  }

  _ecm.Each<components::Imu,
            components::WorldPose,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &_entity,
        const components::Imu * /*_imu*/,
        const components::WorldPose *_worldPose,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Custom gravity
          if (this->sourceModel.Valid(_ecm))
          {
            // world pose of entity
            const auto &X_WB = _worldPose->Data();

            // position vector from central mass to body
            auto p_CB_W = X_WB.Pos() - X_WC.Pos();

            // length of position vector
            double r = p_CB_W.Length();

            // avoid singularity as r -> 0
            if (std::abs(r) < 1.0E-3)
            {
              return true;
            }

            // inverse radius
            double one_over_r = 1.0 / r;
            double r_expo = pow(r, this->gravity_expo);

            // magnitude of acceleration
            auto a = 1.0 * this->G * mass_C * r_expo;

            // direction of acceleration
            auto direction = -1.0 * one_over_r * p_CB_W;

            // acceleration
            auto a_B_W = a * direction;

            it->second->SetGravity(a_B_W);

            // gzdbg << "Gravity: " << a_B_W << "\n";
          }

          const auto &imuWorldPose = _worldPose->Data();
          it->second->SetWorldPose(imuWorldPose);

          // Set the IMU angular velocity (defined in imu's local frame)
          it->second->SetAngularVelocity(_angularVel->Data());

          // Set the IMU linear acceleration in the imu local frame
          it->second->SetLinearAcceleration(_linearAccel->Data());
         }
        else
        {
          gzerr << "Failed to update IMU: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::RemoveImuEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ImuPrivate::RemoveImuEntities");
  _ecm.EachRemoved<components::Imu>(
    [&](const Entity &_entity,
        const components::Imu *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing IMU sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(Imu, System,
  Imu::ISystemPreUpdate,
  Imu::ISystemPostUpdate,
  Imu::ISystemConfigure
)

GZ_ADD_PLUGIN_ALIAS(Imu, "gz::sim::systems::Imu")
