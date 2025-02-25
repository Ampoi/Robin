import RosLib from "roslib";

const RosWebsocketUrl = "ws://192.168.48.189:9090";
export function createRos() {
  const ros = new RosLib.Ros({
    url: RosWebsocketUrl
  });

  ros.on("connection", () => {
    console.log("🙌 Connected to WebSocket");
  });

  ros.on("error", (error) => {
    console.log("⚠ Error occurred in WebSocket Connection: ", error);
  });

  ros.on("close", () => {
    console.log("👋 WebSocket Closed!");
  });

  return { ros }
}
