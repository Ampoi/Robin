import RosLib from "roslib";
import { getLocalStorage } from "../utils/useLocalStorage";

export function createRos() {
  const RosWebsocketUrl = getLocalStorage("WebSocketURL");
  if( !RosWebsocketUrl ) throw new Error("WebSocketURL not found in LocalStorage");
  
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
