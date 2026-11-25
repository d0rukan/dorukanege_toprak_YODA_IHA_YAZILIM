import asyncio
import math
from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.mission import MissionAsync, MissionItem, MissionPlan
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

METERS_PER_DEG_LAT = 111111.0


async def wait_for_altitude(
    telemetry: TelemetryAsync,
    target_altitude_m: float,
    tolerance_m: float = 1.0,
    timeout_s: float = 30.0,
    ) -> None:
    
    async def _poll() -> None:
        async for pos in telemetry.subscribe_position():
            current_alt = pos.relative_altitude_m
            print(
                f"\rMevcut irtifa: {current_alt:.1f} / {target_altitude_m:.1f} m",
                end="",
                flush=True,
            )
            if current_alt >= target_altitude_m - tolerance_m:
                print("\nHedef kalkış irtifasına ulaşıldı!")
                break

    await asyncio.wait_for(_poll(), timeout=timeout_s)


def build_circle_waypoints(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    altitude_m: float,
    speed_m_s: float,
    step_deg: int,
    start_deg: int = 0,
    clockwise: bool = True,
) -> list[MissionItem]:
    
    meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(center_lat))
    direction = 1 if clockwise else -1
    waypoints: list[MissionItem] = []

    num_steps = 360 // step_deg
    for i in range(num_steps + 1):  
        angle_deg = (start_deg + direction * i * step_deg) % 360
        rad = math.radians(angle_deg)

        north_offset = radius_m * math.cos(rad)
        east_offset = radius_m * math.sin(rad)

        lat = center_lat + (north_offset / METERS_PER_DEG_LAT)
        lon = center_lon + (east_offset / meters_per_deg_lon)

        is_last = i == num_steps
        waypoints.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=altitude_m,
                speed_m_s=speed_m_s,
                is_fly_through=not is_last,
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=float("nan"),
                camera_photo_interval_s=float("nan"),
                acceptance_radius_m=1.2 if is_last else 1.5,
                yaw_deg=float("nan"),
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
        )

    return waypoints


async def draw_eight():
    config = Configuration.create_with_component_type(
        ComponentType.GROUND_STATION
    )
    mavsdk = Mavsdk(config)

    print("Drone'a bağlanılıyor...")
    await mavsdk.add_any_connection("udpin://0.0.0.0:14540")

    drone = await mavsdk.first_autopilot(timeout_s=10.0)
    if drone is None:
        print("Zaman aşımı: Bağlantı kurulamadı.")
        return

    action = ActionAsync(drone)
    telemetry = TelemetryAsync(drone)
    mission = MissionAsync(drone)

    # 1) Tek bir global konum (GPS) oku ve çık
    async for pos in telemetry.subscribe_position():
        base_lat = pos.latitude_deg
        base_lon = pos.longitude_deg
        print(
            f"Lat: {pos.latitude_deg:.6f}, "
            f"Lon: {pos.longitude_deg:.6f}, "
            f"RelAlt: {pos.relative_altitude_m:.2f} m"
        )
        break

    # 2) Tek bir NED konum oku ve çık
    async for pv in telemetry.subscribe_position_velocity_ned():
        p = pv.position
        print(
            f"NED -> North: {p.north_m:.2f} m, "
            f"East: {p.east_m:.2f} m, "
            f"Down: {p.down_m:.2f} m"
        )
        break

    # 3) Global ve home konum sağlığını bekle
    print("Global ve home konum sağlığı bekleniyor...")
    async for health in telemetry.subscribe_health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Konum verileri hazır!")
            break

    radius = 15.0
    altitude = 30.0
    speed = 4.0
    step_deg = 18

    meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(base_lat))

    # 1. Daire merkezi: base konumunun 15 m kuzeyi (böylece 0° noktası base_lat/base_lon olur)
    c1_lat = base_lat + (radius / METERS_PER_DEG_LAT)
    c1_lon = base_lon

    # 2. Daire merkezi: 1. daire merkezinin 30 m (2 * radius) kuzeyi -> teğet noktada buluşurlar
    c2_lat = c1_lat + ((2.0 * radius) / METERS_PER_DEG_LAT)
    c2_lon = base_lon + (0.0 / meters_per_deg_lon)

    mission_waypoints = []
    # İlk daire: 180°'den başlayıp saat yönünde dönerek tekrar 180°'ye (teğet noktaya) gelir
    mission_waypoints.extend(
        build_circle_waypoints(
            center_lat=c1_lat,
            center_lon=c1_lon,
            radius_m=radius,
            altitude_m=altitude,
            speed_m_s=speed,
            step_deg=step_deg,
            start_deg=180,
            clockwise=True,
        )
    )
    # İkinci daire: aynı teğet noktadan (180°) ters yönde dönerek sekiz şeklini tamamlar
    mission_waypoints.extend(
        build_circle_waypoints(
            center_lat=c2_lat,
            center_lon=c2_lon,
            radius_m=radius,
            altitude_m=altitude,
            speed_m_s=speed,
            step_deg=step_deg,
            start_deg=180,
            clockwise=False,
        )
    )

    mission_plan = MissionPlan(mission_waypoints)

    print(f"{len(mission_waypoints)} noktalı görev yükleniyor...")
    await mission.set_return_to_launch_after_mission(True)
    await mission.upload_mission(mission_plan)

    await action.set_takeoff_altitude(altitude)
    print("Arm ediliyor...")
    await action.arm()
    print("Kalkış yapılıyor...")
    await action.takeoff()

    try:
        await wait_for_altitude(
            telemetry=telemetry,
            target_altitude_m=altitude,
            tolerance_m=1.0,
            timeout_s=30.0,
        )
    except asyncio.TimeoutError:
        print("\nUyarı: Hedef irtifaya ulaşma süresi aşıldı, göreve geçiliyor...")

    print("Görev başlatılıyor...")
    await mission.start_mission()

    async for progress in mission.subscribe_mission_progress():
        print(
            f"\rGörev ilerlemesi: {progress.current}/{progress.total}",
            end="",
            flush=True,
        )
        if progress.current >= progress.total and progress.total > 0:
            print("\nGörev tamamlandı, RTL bekleniyor...")
            break


if __name__ == "__main__":
    asyncio.run(draw_eight())
