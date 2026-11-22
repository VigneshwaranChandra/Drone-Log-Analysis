# Where does the autopilot software come from? (evidence + how to verify)

## What the log itself shows (facts)
- Log viewer: RflySim Log Replayer - from the RflySim platform of the Reliable Flight Control
  group, Beihang University (China).
- Field names inside the binary are Chinese pinyin: `dianji` (电机, motor), `lingpian` (零偏, zero bias),
  plus `rol`/`pit`/`yaw`. These are names typed by the firmware developers, not by the viewer.
- Control law is ADRC with an extended state observer - widely used in Chinese flight-control
  research (it originates from Han Jingqing, Chinese Academy of Sciences), rare in ArduPilot/PX4.
- File header `RLog`, packet sync `AC`, 100 us tick: not ArduPilot (.bin), not PX4 (.ulg).

## What it does NOT prove
Pinyin names and a Chinese viewer show the **firmware code base / tool chain** has Chinese origin
or was written by Chinese-speaking developers. They do not prove where the board is designed,
assembled or who owns the IP. An Indian company can license, adapt or co-develop such software.
Public web search for "RFLYSENSE / RFLY Innovations" returned nothing, so the company claims could
not be checked online.

## How to verify (things you can do without anyone's permission)
1. Company: search the MCA portal (mca.gov.in) for RFLY Innovations Pvt Ltd -> CIN, directors,
   registered address, date of incorporation.
2. Certification: the drone's DGCA / QCI type certificate and the Digital Sky listing name the
   flight controller and its manufacturer.
3. Hardware: open the autopilot - MCU/IMU part numbers and PCB silkscreen/date codes; Chinese PCB
   fabs print their own codes.
4. Software: right-click the GCS / log-viewer .exe -> Properties -> Details (company, copyright);
   look at the install folder for Chinese docs/DLLs; run `strings` on a firmware .bin for Chinese text.
5. USB: Device Manager -> the autopilot's USB VID/PID -> look it up (vid.pid lists vendors).
6. Ask in writing: "Who owns the flight-control firmware source code, and can we get the log format
   specification and the ADRC parameter documentation?" A real owner can answer both.

Why it matters: Indian government / defence tenders restrict Chinese components in flight
controllers, communication and software. If the company sells "indigenous" autopilots into such
tenders, the software origin is a material question - handle it through proper channels.
