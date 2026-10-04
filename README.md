# ESP32 closed-loop sprinkler pressure controller

PlatformIO firmware for a classic ESP32-WROOM-32D (`esp32dev`) using Arduino. GPIO25 controls MAX14870 DIR, GPIO26 controls PWM, and GPIO34/ADC1_CH6 reads the pressure divider. Pin mappings are fixed as requested.

## Safety behavior

- Boot drives PWM LOW immediately; mode is MANUAL. AUTO is never restored after reboot and must be explicitly enabled in the UI.
- Sensor voltage outside 0.20–4.80 V stops drive, switches to MANUAL, and faults. Five good readings clear the sensor indication, but AUTO remains off. Manual pulses are blocked during a sensor fault.
- Pressure above 120 PSI invokes 300 ms CLOSE pulses as a safety override, including with AUTO off. OPEN is blocked. The fault clears only below 115 PSI.
- Default setpoint maximum: 100 PSI. Adjust limits for the lowest-rated plumbing component.
- **A throttling valve is not a static pressure regulator or relief valve.** With no flow, it may not limit downstream static pressure. Install rated mechanical regulation/relief hardware where a hard pressure limit is required.
- For a fixed nozzle, pressure is the controlled quantity; a flow meter is not required.
- Valve position is estimated from pulse time only and is not feedback. No hardware has been physically tested or flashed by this build.

## Exact wiring

### Grounds and power

Join these on one common ground rail: 12 V supply negative, ESP32 GND, both MAX14870 GND pins, valve YELLOW, sensor BLACK, and divider bottom. Bridge split breadboard ground rails. Check that 12 V positive is not shorted to ground.

- 12 V positive → MAX14870 VIN. VIN is the input; VM is unused.
- ESP32 is powered by USB. Never connect 12 V to ESP32 5V, 3V3, or GPIO.
- MAX14870 EN is left unconnected (carrier default enabled). FLT is unconnected in this first version.
- GPIO25/P25 → DIR; GPIO26/P26 → PWM; GPIO34/P34 → divider node.

### Valve

- M1 → anode/non-striped end of 1N5819 or 1N5822 → striped/cathode end → valve RED (initial OPEN direction).
- M2 → anode/non-striped end of second diode → striped/cathode end → valve BLUE (initial CLOSE direction).
- Valve YELLOW → common ground.
- `DIR_OPEN_LEVEL` starts HIGH. Verify direction with a brief manual pulse; if backwards, change only this setting and retest.

Pololu documents a 100 kΩ pull-down on PWM and EN on carrier item #2961. PWM therefore defaults low and the driver is enabled by default. Confirm the board is the specified item and the pull-downs are fitted. Substitute boards may differ.

### Sensor divider

- Sensor RED → ESP32 5 V only while a proper 5 V source powers the ESP32.
- Sensor BLACK → common ground; GREEN → 10 kΩ → divider node/GPIO34.
- Divider node → 20 kΩ → ground (two 10 kΩ resistors in series make 20 kΩ).
- Optional 100 nF capacitor from divider node to ground.

The ratio is 20k/(10k+20k)=2/3: 0.5 V sensor output becomes about 0.333 V at GPIO34; 4.5 V becomes 3.000 V. Never apply the undivided sensor output to GPIO34.

## Setup, build, and flash

1. Install PlatformIO Core or PlatformIO IDE.
2. Copy `include/secrets.example.h` to `include/secrets.h`. Add station Wi-Fi credentials if desired; blanks mean AP-only. Change the fallback AP password (minimum 8 characters) before deployment. `secrets.h` is ignored by Git.
3. Build: `pio run`.
4. Connect the ESP32 over USB, identify its serial port, then flash: `pio run -t upload`.
5. Serial monitor: 115200 baud.

The first install of OTA support must use USB/serial. After that, build and upload over the trusted Wi-Fi with `pio run -e esp32ota -t upload`. The `esp32ota` environment uses authenticated OTA and defaults to the current controller IP; update `upload_port` if its DHCP address changes. On Windows, allow inbound TCP and UDP port 3232 only from `LocalSubnet` on the `Private` profile; the uploader pins both sockets to that port. Keep the OTA password in ignored `include/secrets.h` and `platformio.local.ini`, matching values in both files. Do not use the example password on a real network.

The fallback AP starts even if station credentials are used. Its default IP is `192.168.10.1`, kept separate from common `192.168.4.x` home networks so AP+STA routing does not collide. Default SSID is `SprinklerController`; `ChangeMe123` is only a placeholder. Serial prints AP/station IPs. Browse to `http://<controller-ip>/`.

## UI and REST API

The mobile UI displays pressure, target, AUTO/MANUAL, valve motion, fault, ADC values, Wi-Fi and uptime; it refreshes every 500 ms. Controls are target set, a status-aware AUTO toggle, OPEN/CLOSE 100/300 ms, STOP, and a guarded zero-calibration action. STOP also turns AUTO off.

- `GET /api/status`
- `POST /api/setpoint` form field `psi`: finite 0–100, persisted in Preferences
- `POST /api/auto` field `enabled=0|1`
- `POST /api/pulse` fields `direction=open|close`, integer `ms=20..1000`
- `POST /api/stop`
- `POST /api/zero` field `confirmed=1`: stores a zero offset in ESP32 Preferences without reflashing. Requires MANUAL, a stopped/settled valve, valid sensor, no fault, and 20 stable raw samples whose central 80% spans no more than 1 PSI. The median raw reading sets the offset, reducing sensitivity to brief ADC outliers. The UI confirms that the gauge sensor port is vented to atmosphere.

The local UI/API has no user authentication. Keep the AP password private and use a trusted LAN. Mismatched browser Origin requests are rejected; this is not authentication.

## Pressure conversion and calibration

Firmware uses ADC1, 11 dB attenuation, sixteen `analogReadMilliVolts()` samples, and an EMA with alpha 0.20. Conversion:

```text
sensorVoltage = adcVoltage / (2/3)
rawPressurePsi = (sensorVoltage - 0.5) * 37.5
calibratedPressure = rawPressurePsi * PRESSURE_GAIN + PRESSURE_OFFSET_PSI
```

The default gain is 1.0 and default offset is 0.0. The UI's **Zero sensor at atmosphere** action computes and saves an offset persistently in ESP32 Preferences; it is kept across reboot and firmware uploads. Use it only with a gauge-pressure sensor truly vented to atmosphere, correct divider wiring, AUTO off, and the valve stopped. The endpoint refuses unstable samples and corrections larger than 30 PSI. Zeroing corrects only the offset; it does not verify the span or divider ratio.

Compare firmware to a known mechanical gauge at two pressures before enabling closed-loop control:

```text
GAIN = (P2 - P1) / (R2 - R1)
OFFSET = P1 - GAIN * R1
```

If changing gain manually, set it in `src/main.cpp` and rebuild. The zero offset can be set from the UI without reflashing. The sensor voltage fault window is 0.20–4.80 V.

## Control behavior

This slow valve uses adaptive incremental pulses (70 ms for ≤1.5 PSI error, 110 ms for ≤3 PSI, 200 ms for ≤6 PSI, otherwise 325 ms), then waits 400 ms for pressure to settle. Deadband is ±0.75 PSI. Pulse timing is non-blocking and wrap-safe with `millis()`; only brief ADC sample spacing and DIR setup use microsecond waits. Direction reversal waits until pulse completion and settling have elapsed. Periodic and action Serial logs include pressure, target, error, ADC/sensor voltage, mode, motion, and fault.

The ADC uses ADC1, which remains available with Wi-Fi on classic ESP32. Wi-Fi station failure/disconnect leaves the controller AP active. Wi-Fi loss does not itself stop previously enabled control; the HTTP interface becomes unavailable over the lost link, so use the AP or STOP locally.

## Commissioning checklist

1. **Power off:** verify continuity of all listed grounds, no short from 12 V+ to ground, and both diode directions (M1/M2 → non-striped → striped → valve wire).
2. **ESP32/sensor only:** leave 12 V disconnected; power by USB. Measure RED-to-BLACK ≈5 V, GREEN-to-BLACK ≈0.5 V at zero pressure, GPIO34 node ≈0.33 V. Confirm Serial reports sensor ≈0.5 V and pressure ≈0 PSI. Stop if GPIO34 approaches 5 V.
3. **Driver supply:** keep AUTO off; apply 12 V to VIN. The valve must not move at startup.
4. **Direction:** issue OPEN 100 ms, wait for settle, then CLOSE 100 ms. If reversed, change only `DIR_OPEN_LEVEL`. Do not change software and wiring at once.
5. **Calibration:** compare to a known gauge at two pressures and use the equations above.
6. **Closed loop:** with rated plumbing and mechanical relief installed, start at a low target such as 20 PSI, enable AUTO manually, observe each pulse and settling interval. Tune pulse duration, then settle time, then deadband; do not begin with continuous PID.

## Assumptions and physical checks

- Verify actual valve RED/BLUE direction with short pulses; color direction is an assumption from the supplied hardware notes.
- Confirm sensor supply/output, divider resistor values/ratio, and GPIO34 voltage before applying 12 V.
- Confirm pressure ratings for every plumbing component; 100/120 PSI are software defaults only.
- Confirm valve current and driver temperature under the actual 12 V load. Driver published current ratings do not establish compatibility with an unmeasured valve load.
- Confirm the board is Pololu item #2961 with its PWM pull-down. FLT monitoring and full-close homing are not enabled in this first version; homing needs verified end-stop behavior.
- No physical movement, watering, pressure test, or firmware flash has been performed.
