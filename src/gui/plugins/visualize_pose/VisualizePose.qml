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
import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/qml"

Rectangle {
  id: visualizePose
  color: "transparent"
  Layout.minimumWidth: 400
  Layout.minimumHeight: 320
  anchors.fill: parent

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  Connections {
    target: VisualizePose
    onLockedChanged: {
      lockButton.checked = VisualizePose.Locked()
    }
  }

  Rectangle {
    id: header
    height: lockButton.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey

    RowLayout {
      anchors.fill: parent
      spacing: 0

      Label {
        text: VisualizePose.targetName.empty ? "No entity selected" : VisualizePose.targetName
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 3
      }

      Item {
        height: entityLabel.height
        Layout.fillWidth: true
      }

      ToolButton {
        id: lockButton
        checkable: true
        checked: false
        text: "Lock entity"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: lockButton.checked ? "qrc:/Gazebo/images/lock.svg" :
                                       "qrc:/Gazebo/images/unlock.svg"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: lockButton.checked ? "Unlock target selection"
            : "Lock target selection"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onToggled: {
          VisualizePose.locked = lockButton.checked
        }
      }

      Label {
        id: entityLabel
        text: 'Entity ' + VisualizePose.targetEntity
        Layout.minimumWidth: 80
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }
    }
  }
  ColumnLayout {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right

    GridLayout {
      Layout.fillWidth: true
      columns: 2

      Text {
        text: "Update rate (Hz)"
        color: "dimgrey"
        Layout.row: 0
        Layout.column: 0
        leftPadding: 5
      }
      GzSpinBox {
        id: updateRate
        Layout.fillWidth: true
        Layout.row: 0
        Layout.column: 1
        value: VisualizePose.updateRate
        maximumValue: 1000
        minimumValue: 1
        decimals: 0
        stepSize: 1
        onEditingFinished: VisualizePose.SetUpdateRate(updateRate.value)
      }

      Text {
        text: "Max points"
        color: "dimgrey"
        Layout.row: 1
        Layout.column: 0
        leftPadding: 5
      }
      GzSpinBox {
        id: maxPoints
        Layout.fillWidth: true
        Layout.row: 1
        Layout.column: 1
        value: VisualizePose.maxPoints
        maximumValue: 1000
        minimumValue: 0
        decimals: 0
        stepSize: 10
        onEditingFinished: VisualizePose.SetMaxPoints(maxPoints.value)
      }
    }

    // Bottom spacer
    Item {
      width: 10
      Layout.row: 6
      Layout.column: 0
      Layout.columnSpan: 6
      Layout.fillHeight: true
    }
  }
}
