import QtQuick 2.0
import Machinekit.HalRemote 1.0
import Machinekit.HalRemote.Controls 1.0
import QtQuick.Window 2.0

Row {
  property alias name: led.name
  property alias text: text.text

  spacing: Screen.pixelDensity * 2

  HalLed {
    id: led
    height: Screen.pixelDensity * 8
    width: height
    name: "rx_comm_error"
    halPin.type: HalPin.S32
  }

  Text {
    id: text
    anchors.verticalCenter: parent.verticalCenter
    text: "RX Comm Error:"
    font.pixelSize: Screen.pixelDensity * 6
  }
}
