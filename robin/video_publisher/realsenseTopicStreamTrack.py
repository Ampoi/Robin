import cv2
import asyncio
from av import VideoFrame
from aiortc import VideoStreamTrack

class RealSenseTopicStreamTrack(VideoStreamTrack):
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