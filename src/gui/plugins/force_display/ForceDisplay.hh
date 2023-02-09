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

  /// \brief Filter Size.
  Q_PROPERTY(
    int filterSize
    READ FilterSize
    WRITE SetFilterSize
    NOTIFY FilterSizeChanged
  )

  /// \brief Shape Index.
  Q_PROPERTY(
    int shapeIndex
    READ ShapeIndex
    WRITE SetShapeIndex
    NOTIFY ShapeIndexChanged
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

  /// \brief Shaft Length.
  Q_PROPERTY(
    double shaftLength
    READ ShaftLength
    WRITE SetShaftLength
    NOTIFY ShaftLengthChanged
  )

  /// \brief Shaft Radius.
  Q_PROPERTY(
    double shaftRadius
    READ ShaftRadius
    WRITE SetShaftRadius
    NOTIFY ShaftRadiusChanged
  )

  /// \brief Head Length.
  Q_PROPERTY(
    double headLength
    READ HeadLength
    WRITE SetHeadLength
    NOTIFY HeadLengthChanged
  )

  /// \brief Head Radius.
  Q_PROPERTY(
    double headRadius
    READ HeadRadius
    WRITE SetHeadRadius
    NOTIFY HeadRadiusChanged
  )

  /// \brief Axes Length.
  Q_PROPERTY(
    double axesLength
    READ AxesLength
    WRITE SetAxesLength
    NOTIFY AxesLengthChanged
  )

  /// \brief Axes Radius.
  Q_PROPERTY(
    double axesRadius
    READ AxesRadius
    WRITE SetAxesRadius
    NOTIFY AxesRadiusChanged
  )

  /// \brief Constructor
  public: ForceDisplay();

  /// \brief Destructor
  public: ~ForceDisplay() override;

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Get the topic.
  /// \return Topic, the wrench topic.
  public: Q_INVOKABLE QString Topic() const;

  /// \brief Set the topic.
  /// \param[in] _topic The wrench topic.
  public: Q_INVOKABLE void SetTopic(const QString &_topic);

  /// \brief Notify that the topic has changed
  signals: void TopicChanged();

  /// \brief Get the update rate.
  /// \return The current update rate.
  public: Q_INVOKABLE double UpdateRate() const;

  /// \brief Set the update rate.
  /// \param[in] _updateRate New update rate.
  public: Q_INVOKABLE void SetUpdateRate(double _updateRate);

  /// \brief Notify that the update rate has changed.
  signals: void UpdateRateChanged();

  /// \brief Filter size sets the number of wrenches kept.
  /// \return The current filter size.
  public: Q_INVOKABLE int FilterSize() const;

  /// \brief Set the filter size. If the display has more than
  /// this number, older wrenches are removed.
  /// \param[in] _filterSize Filter size.
  public: Q_INVOKABLE void SetFilterSize(int _filterSize);

  /// \brief Notify that the filter size has changed.
  signals: void FilterSizeChanged();

  public: Q_INVOKABLE int ShapeIndex() const;
  public: Q_INVOKABLE void SetShapeIndex(int _shapeIndex);
  signals: void ShapeIndexChanged();

  public: Q_INVOKABLE QColor Color() const;
  public: Q_INVOKABLE void SetColor(const QColor &_color);
  signals: void ColorChanged();

  public: Q_INVOKABLE double Alpha() const;
  public: Q_INVOKABLE void SetAlpha(double _alpha);
  signals: void AlphaChanged();

  public: Q_INVOKABLE double ShaftLength() const;
  public: Q_INVOKABLE void SetShaftLength(double _shaftLength);
  signals: void ShaftLengthChanged();

  public: Q_INVOKABLE double ShaftRadius() const;
  public: Q_INVOKABLE void SetShaftRadius(double _shaftRadius);
  signals: void ShaftRadiusChanged();

  public: Q_INVOKABLE double HeadLength() const;
  public: Q_INVOKABLE void SetHeadLength(double _headLength);
  signals: void HeadLengthChanged();

  public: Q_INVOKABLE double HeadRadius() const;
  public: Q_INVOKABLE void SetHeadRadius(double _headRadius);
  signals: void HeadRadiusChanged();

  public: Q_INVOKABLE double AxesLength() const;
  public: Q_INVOKABLE void SetAxesLength(double _axesLength);
  signals: void AxesLengthChanged();

  public: Q_INVOKABLE double AxesRadius() const;
  public: Q_INVOKABLE void SetAxesRadius(double _axesRadius);
  signals: void AxesRadiusChanged();

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
