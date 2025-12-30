import cv2
import sys
import numpy as np

def osmo_capture(camera_index=0):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("ERROR: Could not open camera. Exiting.", file=sys.stderr)
        sys.stderr.flush()
        return

    # Set preferred resolution (e.g., 1920x1080) if possible
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print(f"INFO: Camera {camera_index} opened. Waiting for commands...", file=sys.stderr)
    sys.stderr.flush()

    while True:
        try:
            # Wait for a signal from Rust (e.g., a newline character)
            line = sys.stdin.readline()
            if not line: # EOF means parent process closed pipe
                print("INFO: stdin closed. Exiting.", file=sys.stderr)
                sys.stderr.flush()
                break
            
            # Capture frame with retry
            ret, frame = cap.read()
            if not ret:
                # Retry once after a short delay
                import time
                time.sleep(0.1) # 100ms wait
                ret, frame = cap.read()
                if not ret:
                    print("ERROR: Could not read frame from camera after retry.", file=sys.stderr)
                    sys.stderr.flush()
                    continue
            
            # --- Crop the frame ---
            # Assuming a 1080x1920 vertical frame containing a centered 16:9 (1080x608) image.
            # This removes the top and bottom black bars.
            # y_start = (1920 - 608) // 2 = 656
            # y_end = y_start + 608 = 1264
            # We crop frame[y_start:y_end, x_start:x_end]
            frame = frame[656:1264, :]

            # Get cropped frame dimensions and channels
            height, width, channels = frame.shape
            
            # Ensure it's contiguous for tobytes()
            if not frame.flags['C_CONTIGUOUS']:
                frame = np.ascontiguousarray(frame)
            
            frame_bytes = frame.tobytes()
            data_len = len(frame_bytes)

            # --- New Protocol: Length-Prefixed Binary ---
            # 1. Send metadata (width, height, channels) as 4-byte integers each
            sys.stdout.buffer.write(width.to_bytes(4, 'little'))
            sys.stdout.buffer.write(height.to_bytes(4, 'little'))
            sys.stdout.buffer.write(channels.to_bytes(4, 'little'))

            # 2. Send the length of the data as an 8-byte integer
            sys.stdout.buffer.write(data_len.to_bytes(8, 'little'))

            # 3. Send the raw frame data (binary)
            sys.stdout.buffer.write(frame_bytes)
            sys.stdout.flush()
            sys.stderr.flush() # Flush stderr as well, just in case

        except Exception as e:
            print(f"ERROR: An exception occurred: {e}", file=sys.stderr)
            sys.stderr.flush()
            break

    cap.release()
    print("INFO: Camera released. osmo_capture.py finished.", file=sys.stderr)
    sys.stderr.flush()

if __name__ == '__main__':
    # You can pass camera index as an argument if needed: python osmo_capture.py 1
    if len(sys.argv) > 1:
        try:
            index = int(sys.argv[1])
            osmo_capture(index)
        except ValueError:
            print("ERROR: Invalid camera index. Please provide an integer.", file=sys.stderr)
            sys.stderr.flush()
            sys.exit(1)
    else:
        osmo_capture()
