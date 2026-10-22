.. _admt4000_iio:

ADMT4000 IIO wrapper
####################

Overview
********

``iio_admt4000.c`` exposes the :ref:`ADMT4000 <admt4000>` True Power-On
Multiturn Sensor to the libiio IIOD server as an IIO device. It is a thin
adapter: all hardware access goes through the standard Zephyr sensor API
(``sensor_sample_fetch`` / ``sensor_channel_get`` / ``sensor_attr_get`` /
``sensor_attr_set``) on the backing ``adi,admt4000`` device.

The channel and attribute surface mirrors the no-OS ADMT4000 IIO driver
(``drivers/position/admt4000/iio_admt4000.c``) so that the same host-side
tooling works against either target.

Requirements
************

* The ``libiio`` Zephyr module (``CONFIG_ZEPHYR_LIBIIO_MODULE``).
* ``CONFIG_LIBIIO=y`` and an IIOD transport, normally selected with one of
  the ``iiod-console``, ``iiod-usb``, ``iiod-cdc-acm`` or ``iiod-network``
  snippets.
* A devicetree node with compatible ``adi,admt4000-iio`` placed inside the
  ``iio-context`` node.

``CONFIG_ADMT4000_IIO`` defaults to ``y`` once that node is present.

Devicetree
**********

.. code-block:: devicetree

   &arduino_spi {
           status = "okay";
           cs-gpios = <&arduino_header ARDUINO_HEADER_R3_D10 GPIO_ACTIVE_LOW>;

           admt4000: admt4000@0 {
                   compatible = "adi,admt4000";
                   reg = <0>;
                   spi-max-frequency = <2000000>;
                   adi,vdd-variant = <0>;
           };
   };

   / {
           iio-context {
                   admt4000_iio {
                           compatible = "adi,admt4000-iio";
                           io-name = "admt4000";
                           sensor-device = <&admt4000>;
                   };
           };
   };

``io-name`` sets the device name reported by IIOD. ``sensor-device`` is a
required phandle to the backing ``adi,admt4000`` node.

Value representation
********************

The Zephyr sensor API returns values already converted to SI units. IIO
expresses a reading as ``processed = raw * scale``, so every channel
publishes its reading in **micro-units** with a fixed ``scale`` of
``0.000001``:

.. code-block:: none

   raw   = val1 * 1000000 + val2
   scale = 0.000001

Micro-units are used instead of the milli-units of the generic ``iio,sensor``
wrapper because the CORDIC sine/cosine channels are normalized to ``[-1, 1)``
and would otherwise be quantized to three digits. Every ADMT4000 channel
range fits in ``int32_t`` at micro resolution.

.. note::

   This differs from the no-OS driver, which exposes raw *hardware register*
   counts together with a per-channel hardware scale. Here the ``raw``
   attribute is a scaled SI value, because the wrapper only has access to the
   converted output of the sensor API.

Channels
********

.. list-table::
   :header-rows: 1
   :widths: 12 12 20 14 42

   * - IIO id
     - Name
     - Sensor channel
     - Attributes
     - Processed unit
   * - ``rot0``
     - ``turns``
     - ``SENSOR_CHAN_ROTATION``
     - raw, scale
     - turns (fractional part in quarter-turn steps)
   * - ``angl0``
     - ``angle``
     - ``SENSOR_CHAN_ADMT4000_ANGLE``
     - raw, scale
     - degrees, 0 to 360
   * - ``temp0``
     - ``temp``
     - ``SENSOR_CHAN_AMBIENT_TEMP``
     - raw, scale, offset
     - degrees Celsius
   * - ``count1``
     - ``cosine``
     - ``SENSOR_CHAN_ADMT4000_COS``
     - raw
     - normalized CORDIC cosine, ``[-1, 1)``
   * - ``count2``
     - ``sine``
     - ``SENSOR_CHAN_ADMT4000_SIN``
     - raw
     - normalized CORDIC sine, ``[-1, 1)``
   * - ``voltage0``
     - ``radius``
     - ``SENSOR_CHAN_ADMT4000_RADIUS``
     - raw, scale
     - signal radius, mV/V

The channel ids are chosen so that libiio infers the same IIO channel types
the no-OS driver declares explicitly (``IIO_ROT``, ``IIO_ANGL``, ``IIO_TEMP``,
``IIO_COUNT``, ``IIO_VOLTAGE``).

The temperature ``offset`` is always ``0``; the sensor API already returns
absolute temperature. It is present for parity with the no-OS driver.

Sampling behaviour
------------------

Reading channel index 0 (``rot0``) starts a new sample round: it performs one
``sensor_sample_fetch()`` and caches every channel. The remaining channels are
served from that cache, so all six readings within a round are coherent with
each other. Reading any buffer attribute also invalidates the cache.

Device attributes
*****************

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Attribute
     - Access
     - Values
   * - ``conversion_mode``
     - RW
     - ``0`` continuous, ``1`` one-shot
   * - ``conv_sync_mode``
     - RW
     - ``seq_ctrl``, ``start_edge``
   * - ``conv_sync_mode_available``
     - RO
     - space-separated list of the above
   * - ``angle_filter_enable``
     - RW
     - ``0`` disabled, ``1`` enabled
   * - ``h8_corr_src``
     - RW
     - ``factory``, ``user``
   * - ``h8_corr_src_available``
     - RO
     - space-separated list of the above
   * - ``eck_type``
     - RW
     - ``always_flag``, ``valid``, ``disable_check``
   * - ``eck_type_available``
     - RO
     - space-separated list of the above
   * - ``harmonics``
     - RO
     - eight raw register values: ``h1mag h1ph h2mag h2ph h3mag h3ph h8mag h8ph``
   * - ``harmonics_magnitude_scale``
     - RO
     - ``0.009046``
   * - ``harmonics_phase_scale``
     - RO
     - ``0.087891``
   * - ``coil_reset``
     - WO
     - any value; triggers a GMR coil reset pulse

.. note::

   ``harmonics`` is **read-only** here, unlike in the no-OS driver. The
   ADMT4000 sensor driver's ``attr_get`` path returns raw register values,
   but its ``attr_set`` path (``admt4000_set_converted_hmag_config`` /
   ``admt4000_set_converted_hphase_config``) expects pre-scaled "converted"
   units and is not the inverse of the getter, so a raw value written back
   through it would not round-trip. Making this writable requires a raw
   setter in the sensor driver first.

Not supported
*************

The Zephyr libiio backend (``modules/lib/libiio/zephyr/backend.c``)
implements only ``create``, ``read_attr``, ``write_attr`` and
``get_trigger`` - it has no ``open_buffer`` or ``readbuf`` hook. Therefore:

* **Buffered / streaming capture is not available.** Readings are obtained
  through the per-channel ``raw`` attributes. The no-OS ``submit`` and
  ``trigger_handler`` paths, and ``iio_admt4000_trig.c``, have no counterpart.

  Note that ``iio_info`` nevertheless reports ``admt4000 (buffer capable)``
  and ``1 buffer found: buffer0 (input)``. This is misleading - see
  `Buffered reads fail even though iio_info says "buffer capable"`_.
* **Debug register access is not available.** The backend exposes no
  ``reg_read`` / ``reg_write``, and the sensor driver does not export raw
  register accessors.

The following no-OS attributes are also omitted because the ADMT4000 Zephyr
sensor driver exposes no equivalent: ``faults``, ``gpio0_raw`` through
``gpio5_raw``, ``reinitialize``, and the debug ``reset``. Adding them
requires new ``SENSOR_ATTR_ADMT4000_*`` entries in the sensor driver.

Building and running
********************

A ready-made overlay for the ADI SDP-K1 ships with the iiod sample:

.. code-block:: console

   west build -b adi_sdp_k1/stm32f469xx -S iiod-console -d build-admt4000 \
       modules/lib/libiio/zephyr/samples/iiod -- \
       -DEXTRA_DTC_OVERLAY_FILE=admt4000.overlay \
       -DEXTRA_CONF_FILE=admt4000.conf
   west flash -r openocd -d build-admt4000 --config "$PWD/openocd-sdpk1.cfg"

The ``-S iiod-console`` snippet is what selects ``CONFIG_LIBIIO=y`` and
routes IIOD over the console UART. Without it the image builds fine but
contains no IIO layer at all.

A plain ``west flash`` does not work on this board - see
`Flashing and debugging`_ below.

The overlay targets the EVAL-ADMT4000ARD1Z Arduino shield and mirrors
``zephyr/samples/sensor/admt4000/boards/adi_sdp_k1.overlay``:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Signal
     - Arduino pin
     - STM32F469 pin
   * - SPI1 SCK
     - D13
     - PB3
   * - SPI1 MISO
     - D12
     - PB4
   * - SPI1 MOSI
     - D11
     - PA7
   * - SPI1 CS
     - D10
     - PA15
   * - COIL_RS
     - D4
     - PG9
   * - V_EN (shield rail, ``gpio-hog``)
     - A3
     - PC1
   * - SHDN_N (shield rail, ``gpio-hog``)
     - A1
     - PA4

The ``busy``, ``cnv`` and ``acalc`` GPIOs are optional in the binding and are
not wired by this overlay. The two shield rails are mandatory - see
`The eval-board power rails must be enabled`_.

Once running, connect with libiio **v1.0 or newer** over the serial console:

.. code-block:: console

   iio_info -u "serial:/dev/ttyUSB0,230400,8n1n"
   iio_attr -u "serial:/dev/ttyUSB0,230400,8n1n" -c admt4000 angl0 raw
   iio_attr -u "serial:/dev/ttyUSB0,230400,8n1n" -d admt4000 conv_sync_mode

On Windows the SDP-K1 DAPLink exposes the console as a COM port, so use
``serial:COM22,230400,8n1n`` instead. Note the trailing ``n`` (flow control):
it is part of the libiio v1.0 serial URI syntax. A v0.x client will not work
at all - see `iio_info times out`_.

The overlay overrides the console UART to **230400** baud
(``&uart5 { current-speed = <230400>; }``); the board default is 115200.
IIOD shares that UART, so the host URI, any serial terminal, and the
devicetree value must all agree - a mismatch looks exactly like a hung
board, since a wrong-baud client gets only framing errors back.

Flashing and debugging
**********************

The SDP-K1's stock Zephyr OpenOCD config does not work out of the box.
``boards/adi/sdp_k1/support/openocd.cfg`` does
``source [find board/st_nucleo_f4.cfg]``, which selects an ST-LINK (hla)
adapter - but the SDP-K1 has **no ST-LINK**. Its on-board probe runs DAPLink
firmware and enumerates as CMSIS-DAP, so a plain ``west flash`` fails with:

.. code-block:: none

   Info : The selected transport took over low-level target control.
   Info : clock speed 2000 kHz
   Error: open failed

``openocd-sdpk1.cfg`` in the workspace root fixes this by selecting
``interface/cmsis-dap.cfg`` instead. Pass it with ``--config``:

.. code-block:: console

   west flash -r openocd -d build-admt4000 --config "$PWD/openocd-sdpk1.cfg"
   west debug -r openocd -d build-admt4000 --config "$PWD/openocd-sdpk1.cfg"

That is all that is needed when the Zephyr SDK is installed, since the SDK
ships its own OpenOCD along with the ``interface/`` and ``target/`` scripts
this config sources.

Using the STM32CubeIDE OpenOCD instead
--------------------------------------

If you do not have the Zephyr SDK, the OpenOCD bundled with STM32CubeIDE
works with the same config. Note that the binary and its script directory
live in two *different* CubeIDE plugins, so both ``--openocd`` and
``--openocd-search`` are required. The plugin directory names embed version
numbers that change between CubeIDE releases - the ones below are from
CubeIDE as installed here, so check your own ``plugins/`` directory:

.. code-block:: console

   CUBE=/c/stm32cubeide/STM32CubeIDE/plugins
   OCD_BIN="$CUBE/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855/tools/bin/openocd.exe"
   OCD_SCR="$CUBE/com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929/resources/openocd/st_scripts"

   west flash -r openocd -d build-admt4000 \
       --openocd "$OCD_BIN" --openocd-search "$OCD_SCR" \
       --config "$PWD/openocd-sdpk1.cfg"

To discover the correct paths on your machine:

.. code-block:: console

   ls -d /c/stm32cubeide/STM32CubeIDE/plugins/*openocd*/

Expect two hits - the one containing ``externaltools`` holds the binary, the
other holds ``resources/openocd/st_scripts``.

Debugging from VS Code
----------------------

``.vscode/launch.json`` in the workspace root provides ready-made
`Cortex-Debug <https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug>`_
configurations that wrap the same OpenOCD config:

* **Debug admt4000 (Zephyr SDK OpenOCD)** - builds, flashes, and breaks at
  ``main``. Use this one.
* **Attach admt4000 (Zephyr SDK OpenOCD)** - attaches to already-running
  firmware without reflashing.
* **Debug admt4000 (STM32CubeIDE OpenOCD)** - fallback if the Zephyr SDK is
  not installed.

Install the ``marus25.cortex-debug`` extension, then pick a configuration in
the Run and Debug view (``Ctrl+Shift+D``) and press ``F5``. The launch
configs reference absolute paths to OpenOCD, ``arm-zephyr-eabi-gdb`` and the
STM32F469 SVD file, so adjust them if your SDK or CubeIDE lives elsewhere.

The SVD file enables the Cortex-Debug peripheral register view, which is
useful for inspecting SPI1 and the GPIO ports while debugging the ADMT4000
wiring.

.. note::

   OpenOCD binds GDB to port 3333. If a launch fails with a port-in-use
   error, a previous OpenOCD is still running - kill it first
   (``taskkill /IM openocd.exe /F`` on Windows).

``CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP=y`` is required
------------------------------------------------------

``admt4000.conf`` sets ``CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP=y``. Without
it, flashing appears to succeed but OpenOCD then fails the post-flash
``reset run``:

.. code-block:: none

   wrote 65536 bytes from file zephyr.hex in 3.29s
   Info : SWD DPIDR 0x2ba01477
   Error: Failed to read memory at 0xe000ed04
   Error: [stm32f469.cpu] Examination failed

The tell-tale detail is that the **DP** still answers (``DPIDR`` reads fine)
while every **AP**/memory access fails. That is the signature of the core
having entered ``WFI`` in Zephyr's idle thread: the STM32 gates the debug
clock in sleep and the AHB-AP stops responding. The Kconfig option makes the
SoC init code set ``DBGMCU_CR`` DBG_SLEEP/STOP/STANDBY, which keeps the debug
clock running. You can verify it on target - ``mdw 0xE0042004`` should read
``0x00000007`` once the firmware has booted.

Drop the option for production builds; it costs a small amount of idle
current.

Troubleshooting
===============

``iio_info`` times out
----------------------

.. code-block:: none

   ERROR: sp_blocking_read_next has timedout
   ERROR: READ LINE: -138
   Unable to create IIO context serial:COM22,115200,8n1: timed out (138)

This is almost always a **client/server protocol mismatch**, not a driver
fault. The Zephyr IIOD server is libiio **v1.0**, which speaks a *binary*
protocol. libiio **v0.x** clients speak the old *text* protocol, so the
server never replies and the client times out.

Check the client version first:

.. code-block:: console

   iio_info -V        # must report 1.0 or newer

On Windows the v0.x ``iio_info.exe`` installed by PlutoSDR-M2K lives in
``C:\Windows\System32`` and will shadow any newer build on ``PATH``; invoke
the v1.0 binary by absolute path, or reorder ``PATH``.

Note that libiio v1.0 serial URIs take an extra flow-control character:

.. code-block:: console

   iio_info -u "serial:COM22,230400,8n1n"

A quick way to tell the firmware apart from the client: open the port in a
terminal and send a newline. A v1.0 server stays silent (it expects binary
framing), which looks identical to a hung board - so confirm the board is
alive by resetting it and watching for the Zephyr boot banner instead.

``iio_readdev`` times out while ``iio_info`` works
--------------------------------------------------

libiio v1.0 **replaced** ``iio_readdev`` and ``iio_writedev`` with a single
``iio_rwdev``. Because the names differ, a v1.0 install does *not* overwrite
the old executables - the stale v0.26 ``iio_readdev.exe`` is left behind in
``C:\Windows\System32`` next to the upgraded v1.0 tools. It then fails with
the v0.x protocol-mismatch timeout described above, even though ``iio_info``
in the same directory works:

.. code-block:: console

   iio_attr --version      # 1.0.0
   iio_readdev --version   # 0.26   <- leftover, delete it
   iio_rwdev --version     # 1.0.0  <- the v1.0 replacement

Use ``iio_rwdev``, not ``iio_readdev``. On this driver, however, neither
will stream - see the next section.

Buffered reads fail even though ``iio_info`` says "buffer capable"
------------------------------------------------------------------

.. code-block:: none

   ERROR: iio:device0: Unable to create buffer: No such device (19)
   ERROR: iio:device0: Unable to create stream: No such device (19)

This is expected, and is **not** a wiring or configuration fault.

``iio_info`` advertises a buffer because the generic Zephyr sensor shim
(``modules/lib/libiio/zephyr/drivers/iio_device/sensor.c``) marks every
channel ``scan_element = true``. That puts scan elements in the context XML,
from which the *client* infers a ``buffer0``. Nothing on the device backs it:
``zephyr_create_context()`` only calls ``iio_context_add_device()`` and
``iio_device_add_channels()``, never adding a buffer object. So when IIOD
handles the client's ``OPEN_BUFFER`` command,
``iio_device_get_buffer(dev, 0)`` returns ``NULL`` and it answers
``-ENODEV`` (``iiod/responder.c``). It never even reaches
``iio_buffer_open()``, which would have returned ``-ENOSYS`` for the missing
``open_buffer`` op.

Poll the ``raw`` attribute instead:

.. code-block:: console

   iio_attr -u "serial:COM22,230400,8n1n" -c admt4000 angl0 raw

With a single attribute named, ``iio_attr`` prints just the value, so it
drops straight into a loop. Multiply by the channel ``scale`` (1e-6 for
``angl0``) to get degrees.

The eval-board power rails must be enabled
------------------------------------------

The EVAL-ADMT4000ARD1Z shield's coil-driver supply is controlled by two
Arduino header pins, **not** by the ADMT4000 itself:

===========  ==========  =========================
Arduino pin  SDP-K1 pin  Function
===========  ==========  =========================
A3           PC1         ``V_EN`` - coil driver rail
A1           PA4         ``SHDN_N`` - shutdown, active low
===========  ==========  =========================

Both must be driven high before the sensor is usable. The overlays do this
with ``gpio-hog`` entries, which run at
``CONFIG_GPIO_HOGS_INIT_PRIORITY`` (41), well ahead of the sensor's init
priority (90). ``CONFIG_GPIO_HOGS`` enables itself automatically when hogs
are present in the devicetree.

Omitting the hogs produces this at boot:

.. code-block:: none

   <err> admt4000: CRC mismatch - expected 0x12, got 0x00
   <err> admt4000: ECC1 read failed at index 7: -77
   <err> admt4000_iio: ADMT4000 device admt4000@0 is not ready

The ``got 0x00`` is the giveaway: an unpowered shield never drives MISO, so
the whole SPI response reads back as zeros and fails the CRC check with
``-EBADMSG`` (77). The IIO wrapper then correctly refuses to register with
``-ENODEV`` (19), and the device is absent from ``iio_info``.

Distinguishing "no data" from "wrong data"
------------------------------------------

An all-zero or all-``0xFF`` SPI response means **nothing is driving MISO**,
which rules out CRC-algorithm, SPI-mode, and bit-order theories - those
corrupt data rather than eliminate it. To tell a floating line from one that
is actively held low, toggle the MCU's internal pull resistor on the MISO
pin while halted, and watch ``GPIOx_IDR``:

* Follows the pull both ways - the line is **floating** (device unpowered,
  absent, or wired to a different connector).
* Stays stuck regardless of the pull - something is **driving** it.

Inspecting driver state without a console
-----------------------------------------

Every Zephyr device records its init result. When logging is unavailable
(the IIOD sample leaves ``CONFIG_LOG`` off, and IIOD shares UART5 with the
console so enabling it corrupts the data stream), read the status directly
in GDB:

.. code-block:: none

   52 admt4000@0    init=1 res=77   <- EBADMSG, probe failed
   53 admt4000_iio  init=1 res=19   <- ENODEV, dependency not ready
   54 gpio@40022800 init=1 res=0    <- OK

``res`` is the errno returned by the init function; decode it against
``zephyr/lib/libc/minimal/include/errno.h``.

For deeper digging, build a throwaway diagnostic image with logging and
without optimization, so locals stay readable and driver errors print:

.. code-block:: console

   west build -p -b adi_sdp_k1/stm32f469xx -S iiod-console -d build-dbg \
       modules/lib/libiio/zephyr/samples/iiod -- \
       -DEXTRA_DTC_OVERLAY_FILE=admt4000.overlay \
       -DEXTRA_CONF_FILE=admt4000.conf \
       -DCONFIG_DEBUG_OPTIMIZATIONS=y -DCONFIG_LOG=y \
       -DCONFIG_LOG_MODE_IMMEDIATE=y -DCONFIG_SENSOR_LOG_LEVEL_DBG=y

Use this image for diagnosis only - reflash the normal build before testing
IIOD, since log output on UART5 corrupts the binary protocol stream.

.. note::

   UART consoles retain buffered text across resets. When capturing boot
   output, reset the board *while* the capture is running and confirm the
   log begins at the Zephyr banner - otherwise stale pre-fix errors can look
   like the fix failed.
