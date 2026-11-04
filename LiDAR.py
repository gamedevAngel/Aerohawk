import math
import time
from dataclasses import dataclass
from typing import List, Optional
import rplidar

# LiDAR will be offseted so adjust as needed, 
ANGLE_OFFSET = 0.0

# Range between 0-63, Ignore readings below this, low quality points are noisy/unreliable.
MIN_QUALITY = 10

# Ignore readings at distance 0, these mean "no valid return" on most RPLiDAR units.
MIN_DISTANCE_MM = 1.0

# Fixed by the RPLiDAR C1 hardware: do not change.
LIDAR_BAUDRATE = 460800

@dataclass
class LidarPoint:
    angle: float      
    distance: float   
    quality: int      

    @property
    def x(self) -> float:
        return self.distance * math.cos(math.radians(self.angle))

    @property
    def y(self) -> float:
        return self.distance * math.sin(math.radians(self.angle))

def normalize_angle(angle_deg: float) -> float:
    """Wrap an angle to the range (-180, 180], so 'behind the robot' doesn't
    show up as both -179 and 359 depending on which side you approach from."""
    return (angle_deg + 180) % 360 - 180

class Lidar:
    def __init__(self, port: str):
        self.device = rplidar.RPLidar(port, baudrate=LIDAR_BAUDRATE)
        self.last_scan: List[LidarPoint] = []

    def get_scan(self) -> List[LidarPoint]:
        raw_scan = next(self.device.iter_scans())

        points = []
        for quality, angle, distance in raw_scan:
            if quality < MIN_QUALITY or distance < MIN_DISTANCE_MM:
                continue
            #weird calculation due to akward spot in between 359 and -1 mathmathically ruining everythinng even tho
            #its the same spot so we offset it by 180 and then wrap it back to -180 to 180
            corrected_angle = (angle + ANGLE_OFFSET + 180) % 360 - 180
            points.append(LidarPoint(corrected_angle, distance, quality))

        self.last_scan = points
        return points

    #nearest obstace is set to as cone shape 60 degrees infront, adjust as needed
    def nearest_obstacle(
        self,
        angle_min: float = -30.0,
        angle_max: float = 30.0,
        scan: Optional[List[LidarPoint]] = None,
    ) -> Optional[LidarPoint]:
        
        points = scan if scan is not None else self.last_scan
        candidates = [p for p in points if angle_min <= p.angle <= angle_max]
        if not candidates:
            return None
        return min(candidates, key=lambda p: p.distance)

    def close(self): #PLEASE call this before shutting down robot
        self.device.stop()
        self.device.stop_motor()
        self.device.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


if __name__ == "__main__":
    with Lidar(port="/dev/ttyUSB0") as lidar:
        try:
            while True:
                scan = lidar.get_scan()
                closest = lidar.nearest_obstacle(scan=scan)
                if closest:
                    print(
                        f"Nearest obstacle: {closest.distance:.0f}mm "
                        f"at {closest.angle:.1f} degrees "
                        f"(x={closest.x:.0f}, y={closest.y:.0f})"
                        )
                else:
                    print("No obstacles detected in front cone.")
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")