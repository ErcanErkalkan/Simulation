from djitellopy import Tello
import cv2
import numpy as np
import time

# Izgara boyutu (örnek)
GRID_WIDTH = 800
GRID_HEIGHT = 600

# Bilinen renklerin HSV aralıkları (gerektiğinde ayarlayın)
colors_hsv = {
    "purple": ([130, 50, 50], [160, 255, 255]),
    "red": ([0, 120, 70], [10, 255, 255]),
    "black": ([0, 0, 0], [180, 255, 50]),
    "white": ([0, 0, 200], [180, 30, 255]),
    "yellow": ([20, 100, 100], [30, 255, 255]),
    "blue": ([90, 50, 50], [130, 255, 255]),
    "light blue": ([85, 50, 100], [105, 255, 255]),
    "green": ([40, 50, 50], [80, 255, 255])
}

# Renklerin koordinatları (örnek, 15x15 cm karelerin konumu)
coordinates = {
    "purple": (0, 600),
    "red": (0, 300),
    "black": (0, 0),
    "white": (400, 0),
    "yellow": (800, 0),
    "blue": (800, 300),
    "light blue": (800, 600),
    "green": (400, 600)
}

FRAME_WIDTH = 800
FRAME_HEIGHT = 600
MIN_CONTOUR_AREA = 50  # Küçük kareler için alan eşiğini küçülttük

def nothing(x):
    pass

def create_trackbars():
    cv2.namedWindow("HSV Adjustments")
    cv2.createTrackbar("H Lower", "HSV Adjustments", 0, 179, nothing)
    cv2.createTrackbar("H Upper", "HSV Adjustments", 179, 179, nothing)
    cv2.createTrackbar("S Lower", "HSV Adjustments", 50, 255, nothing)
    cv2.createTrackbar("S Upper", "HSV Adjustments", 255, 255, nothing)
    cv2.createTrackbar("V Lower", "HSV Adjustments", 50, 255, nothing)
    cv2.createTrackbar("V Upper", "HSV Adjustments", 255, 255, nothing)

def get_hsv_range_from_trackbars():
    h_lower = cv2.getTrackbarPos("H Lower", "HSV Adjustments")
    h_upper = cv2.getTrackbarPos("H Upper", "HSV Adjustments")
    s_lower = cv2.getTrackbarPos("S Lower", "HSV Adjustments")
    s_upper = cv2.getTrackbarPos("S Upper", "HSV Adjustments")
    v_lower = cv2.getTrackbarPos("V Lower", "HSV Adjustments")
    v_upper = cv2.getTrackbarPos("V Upper", "HSV Adjustments")
    return (h_lower, s_lower, v_lower), (h_upper, s_upper, v_upper)

def adjust_brightness_contrast(frame, brightness=30, contrast=30):
    frame = cv2.convertScaleAbs(frame, alpha=1+(contrast/100), beta=brightness)
    return frame

def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_vals, upper_vals = get_hsv_range_from_trackbars()
    lower = np.array(lower_vals)
    upper = np.array(upper_vals)
    
    detected = []

    # Sabit renkleri ve trackbar'dan alınan aralığı kontrol et
    all_colors = list(colors_hsv.items())
    all_colors.append(("dynamic", (lower.tolist(), upper.tolist())))

    for color, (low, up) in all_colors:
        mask = cv2.inRange(hsv, np.array(low), np.array(up))
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detected.append((color, (cx, cy)))

    if detected:
        return detected[0]  
    return None, None

def move_to_relative(tello, current_x, current_y, target_x, target_y, speed=30):
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    print(f"Hedef: ({target_x},{target_y}), Mevcut: ({current_x},{current_y}) Hareket: ({delta_x},{delta_y})")
    tello.go_xyz_speed(delta_x, delta_y, 0, speed)
    time.sleep(2)
    return target_x, target_y

# Opsiyonel: ArUco Marker Algılama (daha güvenilir konum referansı için)
# ArUco algılama yapmak için ArUco dictionary ve detector parameters gerekir:
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# parameters = cv2.aruco.DetectorParameters_create()
def detect_aruco(frame, dictionary, parameters):
    # Bu fonksiyon örnek amaçlı, aktif kullanmak isterseniz üstte dictionary ve parameters'i oluşturun.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    if ids is not None:
        # Varsayılan olarak ilk bulunan işareti referans alalım
        # Gerçek dünyada konum çözümlemesi için kameranın kalibrasyon bilgisi gerekebilir.
        return ids[0], corners[0]
    return None, None

def main():
    target_point = (600, 400)  # Hedef koordinat
    tello = Tello()

    create_trackbars()

    # Optik akış parametreleri
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    try:
        tello.connect()
        tello.streamon()
        tello.takeoff()
        print("Tello kalktı!")

        current_x, current_y = None, None
        start_time = time.time()

        old_gray = None
        p0 = None

        stable_color_count = 0
        last_detected_color = None
        stable_threshold = 5

        estimated_x, estimated_y = 0.0, 0.0
        scale_factor = 0.05
        prev_time = time.time()

        # (Opsiyonel) ArUco için dictionary ve parametreleri oluşturun:
        # dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # parameters = cv2.aruco.DetectorParameters_create()

        while True:
            frame = tello.get_frame_read().frame
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            frame = adjust_brightness_contrast(frame, brightness=30, contrast=30)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if old_gray is None:
                old_gray = gray.copy()
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

            # Optik akış
            if p0 is not None and len(p0) > 0:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, p0, None, **lk_params)
                if p1 is not None:
                    good_new = p1[st==1]
                    good_old = p0[st==1]

                    dx = 0
                    dy = 0
                    for (new, old) in zip(good_new, good_old):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        dx += (a - c)
                        dy += (b - d)
                    count = len(good_new)
                    if count > 0:
                        dx /= count
                        dy /= count

                        current_time = time.time()
                        dt = current_time - prev_time
                        prev_time = current_time

                        # Piksel hareketini kabaca konuma entegre ediyoruz
                        estimated_x += dx * scale_factor
                        estimated_y += dy * scale_factor

                    p0 = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
                    if p0 is None or len(p0) < 10:
                        p0 = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
                else:
                    p0 = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)

            old_gray = gray.copy()

            color, center = detect_color(frame)

            # Eğer ArUco kullanmak isterseniz aşağıdaki yorum satırlarını açıp entegre edin:
            # marker_id, marker_corners = detect_aruco(frame, dictionary, parameters)
            # if marker_id is not None:
            #     # ArUco'dan konum bilgisi alın (burada örnek amaçlı)
            #     # Kameranın kalibrasyon bilgilerine göre gerçek dünya koordinatını hesaplamak gerek
            #     # Hesapladıktan sonra current_x, current_y güncellenebilir.
            #     pass

            if color is not None and center is not None:
                if color == last_detected_color:
                    stable_color_count += 1
                else:
                    last_detected_color = color
                    stable_color_count = 1

                if stable_color_count >= stable_threshold and color in coordinates:
                    current_x, current_y = coordinates[color]
                    print(f"Algılanan renk: {color}, Konum güncellendi: ({current_x}, {current_y})")
            else:
                # Renk algılanamadığında optik akıştan tahmini pozisyon güncelle
                if current_x is None or current_y is None:
                    current_x = estimated_x
                    current_y = estimated_y
                else:
                    # Relative hareketi konuma ekleyebilirsiniz.
                    # Ancak burada estimated_x, estimated_y mutlak değil, relative hareket.
                    # Renk tespiti yaptığınızda optik akışı sıfırlayarak hatayı azaltabilirsiniz.
                    current_x += dx * scale_factor
                    current_y += dy * scale_factor

            if current_x is not None and current_y is not None:
                dist_x = target_point[0] - current_x
                dist_y = target_point[1] - current_y
                if abs(dist_x) < 20 and abs(dist_y) < 20:
                    print("Hedefe ulaşıldı veya yeterince yakın!")
                    break
                else:
                    # Hedefe doğru hareket
                    current_x, current_y = move_to_relative(tello, current_x, current_y, target_point[0], target_point[1])

            if time.time() - start_time > 120:
                print("Zaman aşımı: Belirtilen hedefe gidilemedi.")
                break

            cv2.imshow("Tello Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Kullanıcı isteği: Çıkış.")
                break

        tello.land()
        print("Görev tamamlandı, Tello iniş yaptı!")

    except Exception as e:
        print(f"Hata: {e}")
    finally:
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
