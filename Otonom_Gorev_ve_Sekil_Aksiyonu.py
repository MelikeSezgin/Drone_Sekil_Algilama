
import time
import math
import threading

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# =========================================================
CONNECTION_STRING = "127.0.0.1:14550"
CAM_TOPIC = "/iris_demo/gimbal/image_raw"

TAKEOFF_ALT = 10.0

P1_LAT, P1_LON = -35.36326269, 149.16500409

# Seyrüsefer hızı
GOTO_SPEED = 3.5

# kuzey yönünde düz arama
SEARCH_NORTH_SPEED = 2.0
SEARCH_MAX_SEC = 360

# Görsel merkezleme (visual servo)
CENTER_MAX_VEL = 0.9
CENTER_DEADBAND_PX = 25

# Stabil merkezleme
CENTER_TIGHT_PX = 12
CENTER_STABLE_FRAMES = 12


CENTER_MIN_AREA_FRAC_TRI = 0.020
CENTER_MIN_AREA_FRAC_HEX = 0.015

CENTER_TIMEOUT_RED_SEC = 20.0
CENTER_TIMEOUT_HEX_SEC = 8.0


PIX_TO_EAST_SIGN = +1
PIX_TO_NORTH_SIGN = -1


ENABLE_BLUE_HEX_ACTION = True
HEX_DESCEND_ALT = 3.0
HEX_WAIT_SEC = 5


BLUE_COOLDOWN_SEC = 12.0
BLUE_REARM_DIST_M = 6.0
POST_BLUE_ESCAPE_SEC = 2.5


SHOW_DEBUG = True


# =========================================================

class RosCamera:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.sub = rospy.Subscriber(topic, Image, self._cb, queue_size=1)

    def _cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.frame = img
        except Exception as e:
            rospy.logwarn(f"CV Bridge error: {e}")

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()


# =========================================================

def _largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    return max(cnts, key=cv2.contourArea)


def _contour_center(cnt):
    M = cv2.moments(cnt)
    if M["m00"] < 1e-6:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def _approx_vertices(cnt, eps_ratio=0.03):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, eps_ratio * peri, True)
    return len(approx), approx


def detect_shapes(frame_bgr):
    debug = frame_bgr.copy()
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)


    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)


    lower_blue = np.array([95, 120, 70])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=2)

    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=1)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)

    dets = []


    rc = _largest_contour(mask_red)
    if rc is not None:
        area = cv2.contourArea(rc)
        if area > 800:
            vcount, approx = _approx_vertices(rc, eps_ratio=0.035)
            c = _contour_center(rc)
            if c and vcount == 3:
                dets.append({"type": "RED_TRIANGLE", "center": c, "area": float(area)})
                cv2.drawContours(debug, [approx], -1, (255, 255, 255), 2)
                cv2.circle(debug, c, 6, (255, 255, 255), -1)
                cv2.putText(debug, "Kirmizi_Ucgen", (c[0] + 10, c[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


    bc = _largest_contour(mask_blue)
    if bc is not None:
        area = cv2.contourArea(bc)
        if area > 800:
            vcount, approx = _approx_vertices(bc, eps_ratio=0.03)
            c = _contour_center(bc)
            if c and (vcount == 6 or vcount == 7):
                dets.append({"type": "BLUE_HEXAGON", "center": c, "area": float(area)})
                cv2.drawContours(debug, [approx], -1, (255, 255, 255), 2)
                cv2.circle(debug, c, 6, (255, 255, 255), -1)
                cv2.putText(debug, "Mavi_Altigen", (c[0] + 10, c[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    return dets, debug


# =========================================================

def get_distance_metres(a, b):
    dlat = b.lat - a.lat
    dlon = b.lon - a.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5


def send_velocity_local_ned(vehicle, vn, ve, vd, yaw_rate=0.0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vn, ve, vd,
        0, 0, 0,
        0, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def condition_yaw(vehicle, heading_deg, relative=False):
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading_deg,
        0,
        1,
        is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def arm_and_takeoff(vehicle, alt):
    print("Drone Kalkışa Geçiyor...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.2)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)

    vehicle.simple_takeoff(alt)
    while True:
        cur_alt = vehicle.location.global_relative_frame.alt
        if cur_alt >= alt * 0.95:
            break
        time.sleep(0.4)
    print("Drone hedef irtifaya ulaştı.")


def execute_smooth_u_turn(vehicle, duration_sec=3.0, ve=1.0, yaw_rate=0.6):
    print("U dönüşü yapılıyor...")
    t0 = time.time()
    while time.time() - t0 < duration_sec:
        send_velocity_local_ned(vehicle, vn=0.0, ve=ve, vd=0.0, yaw_rate=yaw_rate)
        time.sleep(0.1)
    send_velocity_local_ned(vehicle, 0, 0, 0)


def goto_and_turn(vehicle, target_location):
    vehicle.simple_goto(target_location, groundspeed=GOTO_SPEED)
    while True:
        dist = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        if dist <= 2.0:

            execute_smooth_u_turn(vehicle)
            return
        time.sleep(0.3)


def change_alt(vehicle, target_alt, timeout=50):
    cur = vehicle.location.global_relative_frame
    tgt = LocationGlobalRelative(cur.lat, cur.lon, target_alt)
    vehicle.simple_goto(tgt)
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = vehicle.location.global_relative_frame.alt
        if abs(alt - target_alt) < 0.8:
            return True
        time.sleep(0.3)
    return False


# =========================================================

def center_target_local_ned(vehicle, target_px, frame_w, frame_h):
    cx, cy = target_px
    err_x = cx - frame_w / 2.0
    err_y = cy - frame_h / 2.0

    in_deadband = (abs(err_x) < CENTER_DEADBAND_PX and abs(err_y) < CENTER_DEADBAND_PX)
    in_tight = (abs(err_x) < CENTER_TIGHT_PX and abs(err_y) < CENTER_TIGHT_PX)

    nx = err_x / (frame_w / 2.0)
    ny = err_y / (frame_h / 2.0)

    gain = 0.8
    ve = PIX_TO_EAST_SIGN * CENTER_MAX_VEL * max(-1.0, min(1.0, gain * nx))
    vn = PIX_TO_NORTH_SIGN * CENTER_MAX_VEL * max(-1.0, min(1.0, gain * ny))

    if in_deadband:
        ve *= 0.4
        vn *= 0.4

    def with_min_speed(v, min_abs):
        if abs(v) < min_abs:
            return math.copysign(min_abs, v)
        return v

    if not in_tight:
        ve = with_min_speed(ve, 0.08)
        vn = with_min_speed(vn, 0.08)
    else:
        ve = 0.0
        vn = 0.0

    send_velocity_local_ned(vehicle, vn, ve, 0.0)
    return in_deadband, in_tight


def wait_until_centered(vehicle, cam: RosCamera, shape_type: str,
                        timeout_sec: float,
                        min_area_frac: float,
                        stable_frames: int) -> bool:
    t0 = time.time()
    consec = 0

    while time.time() - t0 < timeout_sec and not rospy.is_shutdown():
        fr = cam.get_frame()
        if fr is None:
            time.sleep(0.02)
            continue

        dets, dbg = detect_shapes(fr)
        same = [d for d in dets if d["type"] == shape_type]
        if not same:
            consec = 0
            send_velocity_local_ned(vehicle, 0, 0, 0)
            if SHOW_DEBUG:
                cv2.imshow("debug", dbg)
                cv2.waitKey(1)
            time.sleep(0.02)
            continue

        same.sort(key=lambda d: d["area"], reverse=True)
        best = same[0]
        c = best["center"]
        h, w = fr.shape[:2]
        area_frac = best["area"] / float(w * h)

        _, in_tight = center_target_local_ned(vehicle, c, w, h)

        if in_tight and area_frac >= min_area_frac:
            consec += 1
        else:
            consec = 0

        if SHOW_DEBUG:
            cv2.imshow("debug", dbg)
            cv2.waitKey(1)

        if consec >= stable_frames:
            send_velocity_local_ned(vehicle, 0, 0, 0)
            return True

        time.sleep(0.04)

    send_velocity_local_ned(vehicle, 0, 0, 0)
    return False


# =========================================================

def straight_north_search(vehicle, cam: RosCamera):
    print("Kuzey yönünde tarama başlıyor..")
    condition_yaw(vehicle, 0, relative=False)
    time.sleep(0.5)

    start = time.time()

    last_blue_time = -1e9
    last_blue_loc = None

    while not rospy.is_shutdown():
        if time.time() - start > SEARCH_MAX_SEC:
            print("Arama süresi doldu -> İnişe geçiliyor...")
            send_velocity_local_ned(vehicle, 0, 0, 0)
            vehicle.mode = VehicleMode("RTL")
            return

        frame = cam.get_frame()
        if frame is None:
            send_velocity_local_ned(vehicle, SEARCH_NORTH_SPEED, 0.0, 0.0)
            time.sleep(0.05)
            continue

        dets, debug = detect_shapes(frame)

        if SHOW_DEBUG:
            cv2.imshow("debug", debug)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                print("Kullanıcı çıkışı -> İnişe geçiliyor... ")
                send_velocity_local_ned(vehicle, 0, 0, 0)
                vehicle.mode = VehicleMode("RTL")
                return


        send_velocity_local_ned(vehicle, SEARCH_NORTH_SPEED, 0.0, 0.0)

        if not dets:
            continue

        dets.sort(key=lambda d: d["area"], reverse=True)
        best = dets[0]
        dtype = best["type"]
        center = best["center"]
        area = best["area"]

        print(f"[DETECT] {dtype} center={center} area={area:.0f}")


        if dtype == "RED_TRIANGLE":
            print("Kırmızı üçgen tespit edildi. Orta noktası hesaplanıyor..")
            send_velocity_local_ned(vehicle, 0, 0, 0)

            ok = wait_until_centered(
                vehicle, cam, "RED_TRIANGLE",
                timeout_sec=CENTER_TIMEOUT_RED_SEC,
                min_area_frac=CENTER_MIN_AREA_FRAC_TRI,
                stable_frames=CENTER_STABLE_FRAMES
            )

            if ok:
                print("Drone üçgenin merkezinde.")
                send_velocity_local_ned(vehicle, 0, 0, 0)
                vehicle.mode = VehicleMode("LAND")
                print("İnişe geçiliyor...")
                return
            else:
                print("[WARN] Üçgen yeterince merkezlenmedi!")
                continue



        if dtype == "BLUE_HEXAGON" and ENABLE_BLUE_HEX_ACTION:
            now = time.time()


            if now - last_blue_time < BLUE_COOLDOWN_SEC:
                continue

            cur_loc = vehicle.location.global_relative_frame
            if last_blue_loc is not None:
                if get_distance_metres(cur_loc, last_blue_loc) < BLUE_REARM_DIST_M:
                    continue

            print("Mavi altıgen bulundu -> Orta noktası hesaplanıyor.")
            send_velocity_local_ned(vehicle, 0, 0, 0)

            ok = wait_until_centered(
                vehicle, cam, "BLUE_HEXAGON",
                timeout_sec=CENTER_TIMEOUT_HEX_SEC,
                min_area_frac=CENTER_MIN_AREA_FRAC_HEX,
                stable_frames=max(6, CENTER_STABLE_FRAMES // 2)  # altıgen için biraz daha hızlı
            )

            if not ok:
                print("[WARN] Altıgen yeterince merkezlenmedi!")
                continue


            change_alt(vehicle, HEX_DESCEND_ALT, timeout=60)
            print("Görev: 3 metreye in, 5 saniye bekle, tekrar 10 metreye çık ve aramaya devam et")
            time.sleep(HEX_WAIT_SEC)


            change_alt(vehicle, TAKEOFF_ALT, timeout=60)

            last_blue_time = time.time()
            last_blue_loc = vehicle.location.global_relative_frame


            t_escape = time.time()
            while time.time() - t_escape < POST_BLUE_ESCAPE_SEC:
                send_velocity_local_ned(vehicle, SEARCH_NORTH_SPEED, 0.0, 0.0)
                time.sleep(0.05)

            continue


# =========================================================

def main():
    rospy.init_node("mission_ros_single", anonymous=True)
    cam = RosCamera(CAM_TOPIC)

    print("Drone'a bağlanılıyor...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)


    home = vehicle.location.global_relative_frame
    start_lat, start_lon = home.lat, home.lon


    arm_and_takeoff(vehicle, TAKEOFF_ALT)

    # Seyrüsefer noktaları
    point1 = LocationGlobalRelative(P1_LAT, P1_LON, TAKEOFF_ALT)
    point2 = LocationGlobalRelative(
        start_lat - (P1_LAT - start_lat),
        start_lon - (P1_LON - start_lon),
        TAKEOFF_ALT
    )


    goto_and_turn(vehicle, point1)
    goto_and_turn(vehicle, point2)


    home_target = LocationGlobalRelative(start_lat, start_lon, TAKEOFF_ALT)
    vehicle.simple_goto(home_target, groundspeed=GOTO_SPEED)

    print("Başlangıç noktasına dönülüyor...")
    while True:
        dist = get_distance_metres(vehicle.location.global_relative_frame, home_target)
        if dist <= 2.0:
            break
        time.sleep(0.3)
    print("Seyrüsefer tamamlandı.")


    straight_north_search(vehicle, cam)

    if SHOW_DEBUG:
        cv2.destroyAllWindows()
    vehicle.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass