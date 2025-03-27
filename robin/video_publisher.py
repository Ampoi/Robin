import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
from aiohttp import web
import cv2
import aiohttp_cors
from aiohttp import web
from rclpy.node import Node
import rclpy

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

class OpenCVCameraStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)  # カメラ番号 0

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("カメラフレームの取得に失敗しました")

        # OpenCVのBGRをRGBへ変換してVideoFrameに変換
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        await asyncio.sleep(1 / 30)  # 30fps程度に制限
        return video_frame


async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection(configuration=RTCConfiguration(
      iceServers=[]
    ))
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
    super().__init__('client')
    print("Video Publisher Node Initialized")

    runServer(on_shutdown, offer)

def main(args=None):
  rclpy.init(args=args)
  
  client = Client()
  rclpy.spin(client)
  
  client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()