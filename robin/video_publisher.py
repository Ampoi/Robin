from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy

import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, VideoStreamTrack
from aiohttp import web
import aiohttp_cors
import cv2
from cv_bridge import CvBridge
from av import VideoFrame

IMAGE_SUBSCRIBE_TOPIC_NAME = "/d455/camera/image_raw"

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

frame = None

class OpenCVCameraStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

    async def recv(self):
        global frame

        pts, time_base = await self.next_timestamp()

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

        self.image_subscriber = self.create_subscription(
            Image, IMAGE_SUBSCRIBE_TOPIC_NAME, self.update_latest_frame, 10
        )

        self.bridge = CvBridge()

        print("Video Publisher Node Initialized")

    def update_latest_frame(self, msg):
        global frame
        last_frame_is_none = frame is None
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        if last_frame_is_none:
            print("Initialize server with the first frame")
            runServer(on_shutdown, offer)

def main(args=None):
    rclpy.init(args=args)

    client = Client()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()