import QtQuick 2.0
import QtQuick.Layouts 1.0
import Machinekit.Controls 1.0
import Machinekit.HalRemote.Controls 1.0
import Machinekit.HalRemote 1.0
import QtQuick.Window 2.0

HalApplicationWindow {
  id: main

  name: "control"
  title: qsTr("P260 Tester")

  Rectangle {
    anchors.fill: parent
    color: "white"
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: Screen.pixelDensity * 2
    spacing: Screen.pixelDensity * 2

    Item {
      Layout.fillHeight: true
    }

    HalButton {
      id: powerOnButton
      Layout.alignment: Layout.Center
      Layout.preferredWidth: Screen.pixelDensity * 60
      Layout.preferredHeight: Screen.pixelDensity * 30
      name: "state_cmd"
      halPin.type: HalPin.U32
      halPin.direction: HalPin.IO
      checkable: true

      HalPin {
        id: resetPin
        name: "reset"
        type: HalPin.HalBit
        direction: HalPin.IO
      }

      Text {
        anchors.centerIn: parent
        text: qsTr("Power On")
        font.pixelSize: Screen.pixelDensity * 6
      }

      Led {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: Screen.pixelDensity * 1
        width: Screen.pixelDensity * 6
        height: width
        onColor: "#00FF00"
        value: powerOnButton.checked
      }
    }

    LedWithText {
      Layout.alignment: Layout.Center
      name: "state_fb"
      text: qsTr("ESTOP Active")
    }

    Item {
      Layout.fillHeight: true
    }
  }
}
