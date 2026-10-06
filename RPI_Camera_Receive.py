import asyncio
import threading
import queue
import websockets
import cv2
import numpy as np
import time

PI_IP = "10.178.125.248"
PORT = 8765

received = queue.Queue(maxsize=1)
stop_event = threading.Event()


def receive_worker():
    async def recv_stream():
        uri = f"ws://{PI_IP}:{PORT}"
        print(f"Connecting to {uri} ...")
        async with websockets.connect(uri, max_size=None, compression=None) as ws:
            print("Connected.")
            while not stop_event.is_set():
                data = await ws.recv()
                try:
                    received.put_nowait(data)
                except queue.Full:
                    try:
                        received.get_nowait()
                    except queue.Empty:
                        pass
                    received.put_nowait(data)

    try:
        asyncio.run(recv_stream())
    except Exception as e:
        print(f"Receive error: {e}")


def display():
    print("Display thread started. Press 'q' in the video window to quit.")

    thread = threading.Thread(target=receive_worker, daemon=True)
    thread.start()

    frame_count = 0
    fps_start_time = time.time()
    fps = 0.0

    try:
        while True:
            try:
                data = received.get(timeout=0.5)
            except queue.Empty:
                continue

            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                print(f"WARNING: decode failed ({len(data)} bytes)")
                continue

            frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_start_time = time.time()
                print(f"Display FPS: {fps:.2f}")

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Drone Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        stop_event.set()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    display()