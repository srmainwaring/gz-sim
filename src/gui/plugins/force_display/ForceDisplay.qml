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
  gz_rviz/gz_rviz_plugins/res/qml/ForceDisplay.qml

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
  id: forceDisplay
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
      placeholderText: "/wrench"
      text: ForceDisplay.topic
      onEditingFinished: {
          ForceDisplay.SetTopic(topic.text)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("The wrench topic.")
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
      value: ForceDisplay.updateRate
      onEditingFinished: {
        ForceDisplay.SetUpdateRate(updateRate.value)
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("The wrench update rate (Hz).")
    }

    // Color
    Text {
      id: colorText
      Layout.columnSpan: 2
      text: "Color"
      color: "dimgrey"
    }
    RowLayout {
      Layout.columnSpan: 2
      Layout.fillWidth: true

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
        text: ForceDisplay.color
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onAccepted: {
          colorDialog.color = color.text
          bgColor.color = color.text
          ForceDisplay.SetColor(color.text)
        }
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Color to draw the arrow.")
      }
    }

    // Alpha
    Text {
      id: alphaText
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
      text: ForceDisplay.alpha.toFixed(1)
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
          ForceDisplay.SetAlpha(alpha.text)
        }
      }
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Amount of transparency to apply to the arrow.")
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
      ForceDisplay.SetColor(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
