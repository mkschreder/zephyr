.. _cobs_uart_async:

COBS UART Async Driver
#######################

Overview
********

The COBS UART Async driver provides a network interface over a UART using:

- **UART Async API** for efficient, interrupt-driven, DMA-based transfers
- **COBS (Consistent Overhead Byte Stuffing)** for reliable framing
- **Point-to-point** networking without hardware flow control

This driver is suitable for applications requiring a serial network link over
two-wire UART (TX/RX only), such as:

- Device-to-device communication
- Gateway connections
- Custom routing stacks

Architecture
************

The implementation consists of three layers:

1. **net_pkt COBS library** (``subsys/net/lib/net_pkt_cobs``)
   
   - Provides ``net_pkt_cobs_encode_inplace()`` and ``net_pkt_cobs_decode_inplace()``
   - Operates directly on ``net_pkt`` structures
   - Reuses the existing COBS library (``lib/utils/cobs.c``)

2. **COBS Serial L2** (``subsys/net/l2/cobs_serial``)
   
   - Network L2 layer that performs COBS encoding/decoding
   - Handles framing with ``0x00`` delimiter
   - Supports optional packet consumer for custom routing

3. **COBS UART Async Driver** (``drivers/net/cobs_uart_async.c``)
   
   - Network device driver using UART Async API
   - Frame delimiter detection and Rx buffering
   - No hardware flow control (ring buffer overflow handling)

Configuration
*************

Kconfig Options
===============

Enable the driver and required components:

.. code-block:: kconfig

   CONFIG_COBS_UART_ASYNC=y
   CONFIG_NET_L2_COBS_SERIAL=y
   CONFIG_NET_PKT_COBS=y
   CONFIG_COBS=y
   CONFIG_UART_ASYNC_API=y

Tuning parameters:

- ``CONFIG_COBS_UART_ASYNC_MTU`` - MTU (default: 1500)
- ``CONFIG_COBS_UART_ASYNC_RX_BUF_LEN`` - UART RX buffer size (default: 128)
- ``CONFIG_COBS_UART_ASYNC_TX_BUF_LEN`` - UART TX buffer size (default: 2048)
- ``CONFIG_COBS_UART_ASYNC_RINGBUF_SIZE`` - Ring buffer for Rx data (default: 512)
- ``CONFIG_COBS_UART_ASYNC_RX_PRIORITY`` - Rx worker priority (default: 7)

Devicetree Configuration
========================

Specify the UART to use via devicetree instance:

.. code-block:: devicetree

   / {
     cobs0: cobs@0 {
       compatible = "zephyr,cobs-uart-async";
       reg = <0>;
       uart = <&uart1>;
       status = "okay";
     };
   };

   &uart1 {
     status = "okay";
     current-speed = <115200>;
   };

The UART device must support the Async API (``CONFIG_UART_ASYNC_API``).

Usage
*****

Basic Network Interface
=======================

Once configured, the driver creates a network interface automatically.
You can interact with it using standard Zephyr networking APIs:

.. code-block:: c

   #include <zephyr/net/net_if.h>
   #include <zephyr/net/net_core.h>

   struct net_if *iface;

   /* Get the COBS interface */
   iface = net_if_get_first_by_type(&NET_L2_GET_NAME(COBS_SERIAL));

   /* Bring it up */
   net_if_up(iface);

   /* Configure IP address */
   struct in_addr addr;
   net_addr_pton(AF_INET, "192.168.7.1", &addr);
   net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);

Custom Packet Consumer
======================

For custom routing stacks, register a consumer callback to intercept
received packets:

.. code-block:: c

   #include <zephyr/net/cobs_serial.h>

   static bool my_packet_consumer(struct net_if *iface, struct net_pkt *pkt)
   {
       /* Process packet in custom routing stack */
       process_custom_packet(pkt);
       
       /* Return true to consume packet (don't pass to IP stack) */
       return true;
   }

   /* Register consumer */
   net_cobs_serial_register_consumer(my_packet_consumer);

Enable the consumer feature in Kconfig:

.. code-block:: kconfig

   CONFIG_NET_L2_COBS_SERIAL_CONSUME_RX=y

Framing Details
***************

The driver uses COBS encoding with the following framing:

- **Delimiter**: ``0x00`` byte
- **Frame format**: ``<COBS-encoded-payload>`` + ``0x00``
- **In-band**: COBS ensures no ``0x00`` bytes appear in encoded payload

This provides:

- Reliable frame synchronization
- No escape sequences needed
- Predictable overhead (1 byte per 254 bytes + framing)

Error Handling
**************

Ring Buffer Overrun
===================

Without hardware flow control, the driver cannot apply backpressure.
If the ring buffer overruns:

1. Current partial frame is dropped
2. Driver resyncs on next ``0x00`` delimiter
3. Warning is logged
4. Statistics counter incremented (if enabled)

To reduce overruns:

- Increase ``CONFIG_COBS_UART_ASYNC_RINGBUF_SIZE``
- Increase Rx worker priority
- Reduce UART baud rate if necessary

Frame Errors
============

Invalid COBS encoding results in:

- Packet dropped
- Error logged
- Statistics updated

API Reference
*************

See:

- :ref:`net_pkt_cobs_api` - COBS encoding/decoding API
- :ref:`cobs_serial_l2_api` - L2 and consumer callback API

Limitations
***********

- No hardware flow control (RTS/CTS)
- Single-threaded Tx (serialized via semaphore)
- MTU limited by buffer sizes

Sample Application
******************

See :ref:`cobs_uart_async_sample` for a complete example.

