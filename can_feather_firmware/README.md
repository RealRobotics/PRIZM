# Can Feather Firmware

This folder contains backups of the firmware for the [Adafruit Feather RP2040 CAN BUS](https://learn.adafruit.com/adafruit-rp2040-can-bus-feather/overview).

To use this, connect the Feather through USB to your computer, copy the ```boot.py``` and ```code.py``` files in the ```CIRCUITPY``` directory and reset.

## Serial ports

`boot.py` enables both USB serial interfaces, so the Feather shows up as two ports:

| Interface | Linux device (typical) | Stable name | Used for |
|---|---|---|---|
| if00 (console) | `/dev/ttyACM0` | `/dev/serial/by-id/usb-Adafruit_Feather_RP2040_CAN_<serial>-if00` | REPL and `print()` output |
| if02 (data) | `/dev/ttyACM1` | `/dev/serial/by-id/usb-Adafruit_Feather_RP2040_CAN_<serial>-if02` | Data link that `code.py` reads and writes |

The `ttyACM` numbers can change (for example to `ttyACM2` after a replug while the old port is still held open), so prefer the `by-id` names. Our board's serial is `DF63CC284F6F2C22`.

When testing by hand with `cat`/`echo`, put the port in raw mode first, or the Linux tty echoes the Feather's output back into it:

```bash
stty -F /dev/ttyACM1 115200 raw -echo
```

Anything that sends a Ctrl-C (`0x03`) to the **console** port stops `code.py` and drops the board to the REPL. Send Ctrl-D to the console port, or press reset, to start it again.

## UDP bridge through the BlueROV2 Navigator (BlueOS)

The Feather can be plugged into a USB port of the Navigator's Raspberry Pi and reached from the topside laptop over UDP:

```
laptop 192.168.2.1  --UDP-->  Pi 192.168.2.2:9000  --USB serial-->  Feather data port (if02)
laptop 192.168.2.1:9001  <--UDP--  Pi                <--USB serial--  Feather
```

From the laptop:

```bash
nc -4u 192.168.2.2 9000
```

Replies go to whoever sent the last packets and also to `192.168.2.1:9001`. With the test sketch, `PING` returns `PONG` and `ECHO <text>` returns `<text>`.

This was set up and tested on BlueOS 1.4.7 (Raspbian 11 host). The files are in [navigator/](navigator).

### From ROS (`hitech_bindings`)

`hitech_bindings` reaches the Feather through `udp_driver` (sender → `192.168.2.2:9000`, receiver on port 9001; see [`udp_config.yaml`](../hitech_bindings/config/udp_config.yaml)):

```bash
ros2 launch hitech_bindings joy_control.launch.py
```
```bash
ros2 launch hitech_bindings udp_driver.launch.py
```

`joy_control.launch.py` uses UDP by default. Add `transport:=serial` for a Feather plugged straight into the machine over USB. `serial_driver.launch.py` still does the same. `hitech_driver` listens on both `serial_read` and `udp_read`, publishes commands on both `/serial_write` and `udp_write`, and re-frames the incoming byte stream, so frames split across or packed into one message are handled.

Raw transport check (with the PING/ECHO test sketch):

```bash
ros2 topic pub --once /udp_write udp_msgs/msg/UdpPacket "{data: [80,73,78,71,10]}"
```

`ros2 topic echo /udp_read` should then show the bytes for `PONG\n`.

### What is installed on the Pi

- **`/usr/local/bin/bridges`**: BlueOS's own serial-to-UDP bridge program, copied out of the `blueos-core` container.
- **[`feather-bridge.service`](navigator/feather-bridge.service)** (`/etc/systemd/system/`): runs `bridges` on the Feather's data port (by-id name, 115200 baud). It restarts every 2 s whenever it exits, so it recovers after the Feather is unplugged and plugged back in. It also starts at boot. Before each start it puts the port in raw mode.
- **[`99-feather-no-probe.rules`](navigator/99-feather-no-probe.rules)** (`/etc/udev/rules.d/`): makes the Feather's ports root-only. Without it, BlueOS's Ping service, which probes every new serial port for Ping sonars, writes bytes to the console port. That stops `code.py` and leaves junk on the data port. The bridge service runs as root, so it is unaffected. On the Pi you need `sudo` to open the Feather's ports.

### Why not the BlueOS "Bridges" page

BlueOS has a built-in serial bridge page, but on 1.4.7:

- it doesn't restart a bridge whose serial device disappears, so a replug kills the link until the bridge is re-added;
- saving the bridge settings fails (`'BridgeSettingsSpecV2' object has no attribute 'model_dump'`), so bridges don't survive a reboot and deleting them can leave orphaned processes.

Don't add a bridge for the Feather there as well, or it will compete with the service for the port. The Bridges page is expected to be empty.

### Install

Copy the files from this folder to the Pi (e.g. `scp navigator/* pi@192.168.2.2:/tmp/`), then on the Pi:

```bash
docker cp blueos-core:/usr/bin/bridges /tmp/bridges
```
```bash
sudo install -m 755 /tmp/bridges /usr/local/bin/bridges
```
```bash
sudo cp /tmp/feather-bridge.service /etc/systemd/system/ && sudo cp /tmp/99-feather-no-probe.rules /etc/udev/rules.d/
```
```bash
sudo udevadm control --reload && sudo systemctl daemon-reload && sudo systemctl enable --now feather-bridge
```

Then unplug and replug the Feather. For a different Feather, change the serial number in the by-id path in `feather-bridge.service`. For a different topside IP, change `192.168.2.1` in the same file.

### Check and troubleshoot

```bash
systemctl status feather-bridge
```
```bash
journalctl -u feather-bridge -f
```

- **"Failed to start" lines while the Feather is unplugged** are normal: the service keeps retrying until it comes back.
- **No reply at all:** check that `code.py` is running by reading the console. It should show `code.py`, not `REPL`:

  ```bash
  sudo sh -c 'stty -F /dev/ttyACM0 115200 raw -echo; timeout 6 cat /dev/ttyACM0'
  ```
- **First message after a replug is lost:** while the port is being opened, a little of the Feather's output can be echoed back into it and gets joined onto the first command. Send it again. The CAN firmware's frame parser resyncs on the next `W`/`w` byte anyway.

### Remove

```bash
sudo systemctl disable --now feather-bridge
```
```bash
sudo rm /etc/systemd/system/feather-bridge.service /etc/udev/rules.d/99-feather-no-probe.rules /usr/local/bin/bridges
```
```bash
sudo udevadm control --reload && sudo systemctl daemon-reload
```
