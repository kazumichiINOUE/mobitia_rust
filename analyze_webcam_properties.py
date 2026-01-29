import cv2

def analyze_webcam_properties(camera_index):
    # 利用可能なバックエンドを試す
    # OpenCV 4.x以降では、特定のバックエンドを指定しないと
    # CAP_ANYが使われるか、デフォルトのバックエンドが自動選択されます。
    # ユーザーの環境に応じて適切なバックエンドをコメントアウト/解除してください。
    # 例: macOSならAVFOUNDATION, LinuxならV4L2, WindowsならDSHOW/MSMF
    
    backends = [
        (cv2.CAP_ANY, "CAP_ANY (Auto)"), # 自動選択
        (cv2.CAP_AVFOUNDATION, "CAP_AVFOUNDATION (macOS)"), # macOS
        (cv2.CAP_V4L2, "CAP_V4L2 (Linux)"),     # Linux
        # (cv2.CAP_DSHOW, "CAP_DSHOW (Windows)"),    # Windows
        # (cv2.CAP_MSMF, "CAP_MSMF (Windows)")      # Windows
    ]

    print(f"Attempting to open camera at index {camera_index} with various backends:")

    for backend_const, backend_name_str in backends:
        print(f"\n--- Trying Backend: {backend_name_str} ---")
        cap = None
        try:
            cap = cv2.VideoCapture(camera_index, backend_const)
            if not cap.isOpened():
                print(f"  Could not open camera with {backend_name_str}")
                if cap:
                    cap.release()
                continue

            # 使用されているバックエンド名を取得 (CAP_ANYの場合、実際に使用されたバックエンド名が返る)
            actual_backend_name = cap.getBackendName()
            
            # ビデオフォーマット(FourCC)を取得
            fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
            # FourCCコードを文字列に変換
            fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
            if fourcc_str.isspace(): # 全て空白の場合の対処
                fourcc_str = "Unknown/Blank"


            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)

            print(f"  Actual Backend Used: {actual_backend_name}")
            print(f"  Success: True")
            print(f"  Resolution: {width}x{height}")
            print(f"  FPS: {fps}")
            print(f"  FourCC: {fourcc_str} ({hex(fourcc_int)})")
            
            # 一フレームだけ読んでみる
            ret, frame = cap.read()
            if ret:
                print(f"  Frame read successful: True, Frame shape: {frame.shape}, Frame type: {frame.dtype}")
            else:
                print(f"  Frame read successful: False (no frame could be read)")

            cap.release()
            print("-" * 20)

        except Exception as e:
            print(f"  An error occurred for {backend_name_str}: {e}")
            if cap:
                cap.release()
        finally:
            if cap and cap.isOpened():
                cap.release()

if __name__ == '__main__':
    # Osmo Pocket 3のカメラ番号に修正してください
    # 0がデフォルトのカメラ、1が2番目のカメラなど
    camera_index = 0
    print(f"Starting analysis for camera index: {camera_index}")
    analyze_webcam_properties(camera_index)
