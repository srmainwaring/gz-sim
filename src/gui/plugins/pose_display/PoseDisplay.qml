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

/*
  Credits:

  QML for display:
  gz_rviz/gz_rviz_plugins/res/qml/PoseDisplay.qml

  Text for ToolTips from:
  rviz_default_plugins/src/rviz_default_plugins/displays/pose/pose_display.cpp

 */

import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.0
import "qrc:/qml"

Rectangle {
  id: poseDisplay
  color: "transparent"
  Layout.minimumWidth: 300
  Layout.minimumHeight: 320
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Light grey according to theme
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  // Dark grey according to theme
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  GridLayout {
    columns: 6
    columnSpacing: 10
    Layout.fillWidth: true
    anchors.fill: parent

    // Left spacer
    Item {
      Layout.columnSpan: 1
      Layout.rowSpan: 15
      Layout.fillWidth: true
    }

    // Topic
    Text {
      id: topicText
      Layout.columnSpan: 2
      color: "dimgrey"
      text: "Topic"
    }

    TextField {
      id: topic
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "/pose"
      text: PoseDisplay.topic
      onEditingFinished: {
          PoseDisplay.SetTopic(topic.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("The pose topic.")
    }

    // Right spacer
    Item {
      Layout.columnSpan: 1
      Layout.rowSpan: 15
      Layout.fillWidth: true
    }

    // UpdateRate
    Text {
      id: updateRateText
      Layout.columnSpan: 2
      text: "Update Rate"
      color: "dimgrey"
    }
    GzSpinBox {
      id: updateRate
      Layout.columnSpan: 2
      Layout.fillWidth: true
      maximumValue: 1000
      minimumValue: 1
      decimals: 0
      stepSize: 1
      value: PoseDisplay.updateRate
      onEditingFinished: {
        PoseDisplay.SetUpdateRate(updateRate.value)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("The pose update rate (Hz).")
    }

    // Filter Size
    Text {
      id: filterSizeText
      Layout.columnSpan: 2
      text: "Filter Size"
      color: "dimgrey"
    }
    GzSpinBox {
      id: filterSize
      Layout.columnSpan: 2
      Layout.fillWidth: true
      maximumValue: 1000
      minimumValue: 1
      decimals: 0
      stepSize: 10
      value: PoseDisplay.filterSize
      onEditingFinished: {
        PoseDisplay.SetFilterSize(filterSize.value)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("The number of pose messages to display.")
    }

    // Shape
    Text {
      id: shapeText
      Layout.columnSpan: 2
      text: "Shape"
      color: "dimgrey"
    }
    ComboBox {
      id: shape
      Layout.columnSpan: 2
      Layout.fillWidth: true
      currentIndex: PoseDisplay.shapeIndex
      model: [ "Arrow", "Axis" ]
      onCurrentIndexChanged: {
        if (currentIndex < 0) {
          return;
        }
        PoseDisplay.SetShapeIndex(currentIndex)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Shape to display the pose as.")
    }

    // Color
    Text {
      id: colorText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Color"
      color: "dimgrey"
    }
    RowLayout {
      Layout.columnSpan: 2
      Layout.fillWidth: true
      visible: shape.currentIndex === 0

      Button {
        id: colorButton
        Layout.preferredWidth: 60
        Layout.preferredHeight: 40
        Layout.fillWidth: true
        onClicked: colorDialog.open()
        background: Rectangle {
          id: bgColor
          color: color.text
          border.color: "#000000"
          border.width: 2
        }
      }
      TextField {
        id: color
        Layout.fillWidth: true
        placeholderText: "placeholder"
        text: PoseDisplay.color
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onAccepted: {
          colorDialog.color = color.text
          bgColor.color = color.text
          PoseDisplay.SetColor(color.text)
        }
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Color to draw the arrow.")
      }
    }

    // Alpha
    Text {
      id: alphaText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Alpha"
      color: "dimgrey"
    }
    TextField {
      id: alpha
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "1.0"
      text: PoseDisplay.alpha.toFixed(1)
      validator: DoubleValidator {
        bottom: 0.0
        top: 1.0
        decimals: 1
      }
      onAccepted: {
        if (parseFloat(alpha.text) >= validator.bottom &&
            parseFloat(alpha.text) <= validator.top) {
          colorDialog.color.a = alpha.text;
          bgColor.color = colorDialog.color
          PoseDisplay.SetAlpha(alpha.text)
        }
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Amount of transparency to apply to the arrow.")
    }

    //  Shaft Length
    Text {
      id: shaftLengthText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Shaft Length"
      color: "dimgrey"
    }
    TextField {
      id: shaftLength
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "1.0"
      text: PoseDisplay.shaftLength.toFixed(1)
      validator: DoubleValidator {
        bottom: 0.0
        top: 10.0
        decimals: 1
      }
      onAccepted: {
        PoseDisplay.SetShaftLength(shaftLength.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Length of the arrow's shaft, in meters.")
    }

    //  Shaft Radius
    Text {
      id: shaftRadiusText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Shaft Radius"
      color: "dimgrey"
    }
    TextField {
      id: shaftRadius
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "0.05"
      text: PoseDisplay.shaftRadius.toFixed(2)
      validator: DoubleValidator {
        bottom: 0.0
        top: 1.0
        decimals: 2
      }
      onAccepted: {
        PoseDisplay.SetShaftRadius(shaftRadius.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Radius of the arrow's shaft, in meters.")
    }

    //  Head Length
    Text {
      id: headLengthText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Head Length"
      color: "dimgrey"
    }
    TextField {
      id: headLength
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "0.3"
      text: PoseDisplay.headLength.toFixed(1)
      validator: DoubleValidator {
        bottom: 0.0
        top: 3.0
        decimals: 1
      }
      onAccepted: {
        PoseDisplay.SetHeadLength(headLength.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Length of the arrow's head, in meters.")
    }

    //  Head Radius
    Text {
      id: headRadiusText
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      text: "Head Radius"
      color: "dimgrey"
    }
    TextField {
      id: headRadius
      visible: shape.currentIndex === 0
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "0.10"
      text: PoseDisplay.headRadius.toFixed(2)
      validator: DoubleValidator {
        bottom: 0.0
        top: 1.0
        decimals: 2
      }
      onAccepted: {
        PoseDisplay.SetHeadRadius(headRadius.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Length of the arrow's head, in meters.")
    }

    // Axes Length
    Text {
      id: axesLengthText
      visible: shape.currentIndex === 1
      Layout.columnSpan: 2
      text: "Axes Length"
      color: "dimgrey"
    }
    TextField {
      id: axesLength
      visible: shape.currentIndex === 1
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "1.0"
      text: PoseDisplay.axesLength.toFixed(1)
      validator: DoubleValidator {
        bottom: 0.0
        top: 100.0
        decimals: 1
      }
      onAccepted: {
        PoseDisplay.SetAxesLength(axesLength.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Length of each axis, in meters.")
    }

    // Axes Radius
    Text {
      id: axesRadiusText
      visible: shape.currentIndex === 1
      Layout.columnSpan: 2
      text: "Axes Radius"
      color: "dimgrey"
    }
    TextField {
      id: axesRadius
      visible: shape.currentIndex === 1
      Layout.columnSpan: 2
      Layout.fillWidth: true
      placeholderText: "0.10"
      text: PoseDisplay.axesRadius.toFixed(2)
      validator: DoubleValidator {
        bottom: 0.0
        top: 10
        decimals: 2
      }
      onAccepted: {
        PoseDisplay.SetAxesRadius(axesRadius.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Radius of each axis, in meters.")
    }

    // Bottom spacer
    Item {
      Layout.columnSpan: 6
      Layout.fillHeight: true
    }
  }

  ColorDialog {
    id: colorDialog
    title: "Select Color"
    color: color.text
    showAlphaChannel: false
    onAccepted: {
      bgColor.color = colorDialog.color
      color.text = colorDialog.color
      colorDialog.color.a = alpha.text
      PoseDisplay.SetColor(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
