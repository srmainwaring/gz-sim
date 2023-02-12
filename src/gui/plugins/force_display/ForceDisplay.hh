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

#ifndef GZ_SIM_GUI_POSEDISPLAY_HH_
#define GZ_SIM_GUI_POSEDISPLAY_HH_

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
namespace gui
{

class ForceDisplayPrivate;

class ForceDisplay : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Topic.
  Q_PROPERTY(
    QString topic
    READ Topic
    WRITE SetTopic
    NOTIFY TopicChanged
  )

  /// \brief Update Rate.
  Q_PROPERTY(
    double updateRate
    READ UpdateRate
    WRITE SetUpdateRate
    NOTIFY UpdateRateChanged
  )

  /// \brief Color.
  Q_PROPERTY(
    QColor color
    READ Color
    WRITE SetColor
    NOTIFY ColorChanged
  )

  /// \brief Alpha.
  Q_PROPERTY(
    double alpha
    READ Alpha
    WRITE SetAlpha
    NOTIFY AlphaChanged
  )

  /// \brief Constructor
  public: ForceDisplay();

  /// \brief Destructor
  public: ~ForceDisplay() override;

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: Q_INVOKABLE QString Topic() const;
  public: Q_INVOKABLE void SetTopic(const QString &_topic);
  signals: void TopicChanged();

  public: Q_INVOKABLE double UpdateRate() const;
  public: Q_INVOKABLE void SetUpdateRate(double _updateRate);
  signals: void UpdateRateChanged();

  public: Q_INVOKABLE QColor Color() const;
  public: Q_INVOKABLE void SetColor(const QColor &_color);
  signals: void ColorChanged();

  public: Q_INVOKABLE double Alpha() const;
  public: Q_INVOKABLE void SetAlpha(double _alpha);
  signals: void AlphaChanged();

  // Documentation inherited
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \internal
  /// \brief Pointer to private data
  private: std::unique_ptr<ForceDisplayPrivate> dataPtr;
};

}  // namespace gui
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_GUI_POSEDISPLAY_HH_
