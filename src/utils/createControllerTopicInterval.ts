import RosLib from "roslib";
import type { Control } from "../model/control";

function convertControlToJoyMessage(controls: Control): RosLib.Message {
  return {
    //右がマイナス
    //上がプラス
    axes: [
      controls.leftStick.x,
      -controls.leftStick.y,
      controls.rightStick.y,
      controls.rightStick.x,
      0,
      0,
      1,
      (controls.RIGHT && controls.LEFT)
        ? 0
        : controls.RIGHT
          ? -1
          : controls.LEFT
            ? 1
            : 0,
      (controls.DOWN && controls.UP)
        ? 0
        : controls.DOWN
          ? -1
          : controls.UP
            ? 1
            : 0,
    ],
    buttons: [
      controls.A ? 1 : 0,
      controls.B ? 1 : 0,
      controls.X ? 1 : 0,
      controls.Y ? 1 : 0,
      controls.LB ? 1 : 0,
      controls.RB ? 1 : 0,
      0,
      0,
      0,
      0,
      0
    ],
  };
}

export function createControllerTopicInterval(ros: RosLib.Ros, TPS: number, controls: Control){
  const joyTopic = new RosLib.Topic({
    ros,
    name: "/joy",
    messageType: "sensor_msgs/Joy",
  })
  return setInterval(() => {
    console.log(JSON.stringify(convertControlToJoyMessage(controls)));
    joyTopic.publish(convertControlToJoyMessage(controls));
  }, 1000 / TPS);
}