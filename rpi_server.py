import asyncio
import io
import threading
import websockets
import picamera2
import time
import argparse
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

RESOLUTIONS = {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
}

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

def parse_args():
    parser = argparse.ArgumentParser(description="RPi camera WebSocket JPEG streamer")
    parser.add_argument(
        "--resolution", choices=RESOLUTIONS.keys(), default="480p",
        help="Output resolution (default: 480p)"
    )
    parser.add_argument(
        "--fps", type=int, default=30,
        help="Capture framerate (default: 30, max depends on resolution)"
    )
    parser.add_argument(
        "--port", type=int, default=8765,
        help="WebSocket port (default: 8765)"
    )
    return parser.parse_args()

async def stream_video(websocket, resolution, fps):
    picam2 = picamera2.Picamera2()

    if fps == 90:
        frame_duration_us = 11111
    else:
        frame_duration_us = 33333

    config = picam2.create_video_configuration(
        main={"size": resolution, "format": "YUV420"},
        buffer_count=4
    )

    picam2.configure(config)

    output = StreamingOutput()
    picam2.start_recording(JpegEncoder(q=50), FileOutput(output))
    picam2.set_controls({"FrameDurationLimits": (frame_duration_us, frame_duration_us)})
    await asyncio.sleep(0.5)

    print(f"Streaming started at {resolution[0]}x{resolution[1]} @ {fps} fps")

    frame_count = 0
    fps_start_time = time.time()
    fps = 0.0

    try:
        while True:
            with output.condition:
                while output.frame is None:
                    output.condition.wait()
                data = output.frame
                output.frame = None
            await websocket.send(data)

            frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_start_time = time.time()
                print(f"FPS: {fps:.2f}")
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected.")
    finally:
        picam2.stop_recording()
        picam2.close()

async def main():
    args = parse_args()
    resolution = RESOLUTIONS[args.resolution]
    fps = args.fps
    port = args.port

    async def handler(websocket):
        await stream_video(websocket, resolution, fps)

    async with websockets.serve(handler, "0.0.0.0", port, max_size=None, compression=None):
        print(f"WebSocket JPEG server started on ws://0.0.0.0:{port} "
              f"({resolution[0]}x{resolution[1]} @ {fps} fps)")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())