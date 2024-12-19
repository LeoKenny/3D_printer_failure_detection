# 3D_printer_failure_detection

Failure detection acquisition, analysis and training

Vibration data is collected from an Ender 3 V2 with a triaxial accelerometer (ADXL345
at 3200 Hz) and used to classify six machine states: nominal, two anomalous extrusion
temperatures (220 °C, 230 °C), a loose X-axis carriage, and two nozzle-obstruction
levels (0.3 mm, 0.2 mm).

See [REPRODUCE.md](REPRODUCE.md) for the end-to-end pipeline, from the raw acquisitions
through to the model results, and `repro/` for the scripts that run it. The environment
is managed by [pixi](https://pixi.sh):

```bash
pixi install
pixi run label && pixi run downsample && pixi run features
pixi run -e train train && pixi run -e train evaluate
```

The original notebooks used to develop the pipeline are kept for reference in
[`notebooks/`](notebooks/); `repro/` reproduces the same steps as scripts that run
unattended end to end.

## Publications

- Leonardo Kenny Treichel da Cunha. *Sistema de aquisição de dados e detecção de falhas
  para impressoras 3D utilizando modelos inteligentes* (Data Acquisition System and Fault
  Detection for 3D Printers Using Intelligent Models). Undergraduate thesis, Universidade
  Federal do Rio Grande do Sul, 2024.
  [lume.ufrgs.br/handle/10183/279376](https://lume.ufrgs.br/handle/10183/279376)
- Leonardo Kenny Treichel da Cunha, Tiago Oliveira Weber. *Data Acquisition System and
  Fault Detection for 3D Printers Using Intelligent Models*. INSCIT 2026. (Reference to
  follow on publication.)

## Transfer SPI (Arduino[Slave]-Raspberry[Master])

[Esp32 Library](https://github.com/hideakitai/ESP32SPISlave/tree/main)

[Raspberry pi Library](https://abyz.me.uk/rpi/pigpio/cif.html)

Installing pigpio

```bash
sudo apt-get update
sudo apt-get install pigpio python-pigpio python3-pigpio
```
