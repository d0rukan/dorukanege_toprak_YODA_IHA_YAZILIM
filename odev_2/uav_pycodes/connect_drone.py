import asyncio
import inspect
from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

async def readTele(func):
   data = func()
   if inspect.iscoroutine(data):
      data = await data
   return data
   
async def run():
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

    await action.set_takeoff_altitude(20.0)
 
    print("Motorlar arm ediliyor...")
    await action.arm()

    print("10 metreye kalkış yapılıyor...")
    await action.takeoff()

    await asyncio.sleep(8)

    # Önce coroutine'i await edip async iterator'ı alıyoruz, sonra anext ile ilk veriyi çekiyoruz
    pos = await readTele(telemetry.position)

    print(
        f"pos -> raw: {pos}, "
        f"Konum -> Enlem: {pos.latitude_deg:.6f}, "
        f"Boylam: {pos.longitude_deg:.6f}, "
        f"Relatif İrtifa: {pos.relative_altitude_m:.2f} m"
    )
   

    print("RTL (Kalkış Noktasına Dönüş) başlatılıyor...")
    await action.return_to_launch()

    await asyncio.sleep(10)
    print("RTL komutu gönderildi, görev tamamlandı.")


if __name__ == "__main__":
    asyncio.run(run())
