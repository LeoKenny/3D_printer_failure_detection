# 3D_printer_failure_detection

Failure detection acquisition, analysis and training

Vibration data is collected from an Ender 3 V2 with a triaxial accelerometer (ADXL345
at 3200 Hz) and used to classify six machine states: nominal, two anomalous extrusion
temperatures (220 °C, 230 °C), a loose X-axis carriage, and two nozzle-obstruction
levels (0.3 mm, 0.2 mm).

See [REPRODUCE.md](REPRODUCE.md) for the end-to-end pipeline, from the raw acquisitions
through to the model results, and `repro/` for the scripts that run it.

## Transfer SPI (Arduino[Slave]-Raspberry[Master])

[Esp32 Library](https://github.com/hideakitai/ESP32SPISlave/tree/main)

[Raspberry pi Library](https://abyz.me.uk/rpi/pigpio/cif.html)

Installing pigpio

```bash
sudo apt-get update
sudo apt-get install pigpio python-pigpio python3-pigpio
```
