from rclpy.node import Node
import rclpy

import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration
from aiohttp import web
import aiohttp_cors
import cv2
import asyncio
from av import VideoFrame
from aiortc import VideoStreamTrack
from cv_bridge import CvBridge


def runServer(on_shutdown, offer):
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_post("/offer", offer)

    cors = aiohttp_cors.setup(
        app,
        defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
            )
        },
    )

    for route in list(app.router.routes()):
        cors.add(route)

    web.run_app(app, port=8080)


logging.basicConfig(level=logging.INFO)
pcs = set()

latest_image_msg = None

class OpenCVCameraStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        try:
          frame = self.bridge.imgmsg_to_cv2(latest_image_msg, desired_encoding='bgr8')
        except Exception as e:
          self.get_logger().error(f"Error converting image: {e}")
          return

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        await asyncio.sleep(1 / 30)  # 30fps程度に制限
        return video_frame


async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=[]))
    pcs.add(pc)

    pc.addTrack(OpenCVCameraStreamTrack())

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


class Client(Node):
    def __init__(self):
        super().__init__("client")
        print("Video Publisher Node Initialized")

        self.image_subscriber = self.create_subscription(
            Image, IMAGE_SUBSCRIBE_TOPIC_NAME, self.image_callback, 10
        )

        print("start webrtc server")
        runServer(on_shutdown, offer)

    def image_callback(self, msg):
        latest_image_msg = msg


def main(args=None):
    rclpy.init(args=args)

    client = Client()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()