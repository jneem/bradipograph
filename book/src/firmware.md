# Firmware and software installation

Bradipograph needs two pieces of software to run: the firmware that gets flashed
onto the bradipograph itself, and the control software that gets run on your
computer.

## Obtain the feeder

Currently, the control software (the "feeder") is only built and tested on x86_64 linux; a statically-linked executable
is available [here](https://github.com/jneem/bradipograph/releases/latest/download/bradipo-feeder). After downloading it
and marking it executable, you should be able to run it:

```
./bradipo-feeder --help


Usage: bradipo-feeder [OPTIONS] [COMMAND]

Commands:
  calibrate        Calibrate the bradipograph from scratch
  set-arm-lengths  Re-calibrate just the arm lengths
  set-max-hang     Set the maximum vertical hang
  set-min-angle    Set the minimum hanging angle
  move             Move to a coordinate
  svg              Draw an SVG file
  square           Draw a square showing the drawable region
  query            Query the current bradipograph status
  help             Print this message or the help of the given subcommand(s)

Options:
      --simulate <SIMULATE>  If set, simulates a bradipograph and draws its output to an svg file at the given path
  -h, --help                 Print help
```

The feeder should also run on Windows and macOS, but you'll need to figure it out yourself;
the source code is [here](https://github.com/jneem/bradipograph/tree/main/crates/feeder).

## Flash the firmware

1. Install `espflash` by following its [installation instructions](https://github.com/esp-rs/espflash/blob/main/espflash/README.md#installation).
2. Download the latest bradipograph [firmware](https://github.com/jneem/bradipograph/releases/latest/download/bradipograph) and the [partition map](https://github.com/jneem/bradipograph/releases/latest/download/partition.csv).
3. Connect your ESP32-C3 (part number 5 in [Materials](./materials.md#other-parts)) to your computer with a USB cable. Your device should
   show up as a serial port; note its name. (On my linux system, it's usually `/dev/ttyACM0`.)
4. Flash your device:
    ```
    espflash flash <path/to/bradipograph> -p </path/to/serial> --partition-table <path/to/partition.csv>
     ```
    where you'll replace `<path/to/bradipograph>` with the location of the bradipograph firmware you downloaded in step 2,
    `<path/to/partition.csv>` with the location of the partition map you downloaded in step 2, and
    `</path/to/serial>` with the serial port that appeared in step 3.
5. With your ESP32-C3 still plugged in, test it by running the feeder. It should find your bradipograph device and see
   that it hasn't been calibrated yet:

   ```
   ./bradipo-feeder query
   calibration: Uncalibrated
   ```

With the firmware set up, you're ready to proceed with [assembly](./assembly.md).
