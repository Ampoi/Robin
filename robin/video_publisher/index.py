import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription
from realsenseTopicStreamTrack import RealSenseTopicStreamTrack
from runServer import runServer
from aiohttp import web

logging.basicConfig(level=logging.INFO)
pcs = set()

async def on_shutdown():
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


class Client(Node):
  def __init__(self):
    super().__init__('client')
    print("Video Publisher Node Initialized")
    
    async def offer(request):
      params = await request.json()
      offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

      pc = RTCPeerConnection()
      pcs.add(pc)

      pc.addTrack(RealSenseTopicStreamTrack())

      await pc.setRemoteDescription(offer)
      answer = await pc.createAnswer()
      await pc.setLocalDescription(answer)

      return web.Response(
        content_type="application/json",
        text=json.dumps(
          {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
      )

    runServer(on_shutdown, offer)

def main(args=None):
  rclpy.init(args=args)
  
  client = Client()
  rclpy.spin(client)
  
  client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()