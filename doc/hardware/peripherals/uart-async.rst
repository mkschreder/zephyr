.. _uart_async_guide:

UART Asynchronous API (Async UART) — implementation and usage guide
###################################################################

This document is an in-depth guide to Zephyr's UART asynchronous API
(:ref:`uart_async_api`). It focuses on:

- How to **implement** async-capable UART drivers correctly.
- How to **use** async UART safely in subsystems and applications.

The guide is written from a "what actually happens" perspective, based on:

- The API contract in ``include/zephyr/drivers/uart.h``.
- The RX helper in ``include/zephyr/drivers/serial/uart_async_rx.h``.
- Driver implementations (DMA + IRQ hybrids) such as:
  STM32, nrfx UARTE, ESP32, ns16550, MCUX LPUART, SiLabs USART.
- Subsystem users such as:
  shell UART backend, modem UART backends, Modbus serial, MCTP UART, EC host
  command UART backend.


Overview: what "async UART" means in Zephyr
******************************************

Async UART is a callback/event driven API for:

- **TX**: one in-flight TX buffer at a time, completion reported by event.
- **RX**: an in-flight RX stream into one or more user-provided buffers,
  with the driver requesting the *next* buffer to avoid gaps.

Typical implementations are DMA-backed (to avoid per-byte interrupts), but the
API is defined in terms of events and buffer ownership; it does not require DMA.

One critical clarification (because it is frequently misunderstood):

- "Async UART" does **not** mean "no interrupts".
  Even DMA-backed drivers typically use interrupts for DMA completion, UART
  error conditions, and/or RX idle/timeout detection. The async API is about the
  *programming model* (callback + events + explicit buffer ownership), not about
  eliminating interrupts entirely.

Important high-level constraints:

- The async callback is typically invoked from **interrupt context** (ISR),
  including DMA completion callbacks which also run in ISR context on most SoCs.
- Async RX/TX and interrupt-driven UART share hardware IRQ machinery.
  Do **not** use both callback models on the same UART instance at the same
  time; see also ``CONFIG_UART_EXCLUSIVE_API_CALLBACKS``.


Callback execution context (ISR) and IRQ-safety
**********************************************

Treat the async callback (registered via :c:func:`uart_callback_set`) as
executing in **interrupt context**, even if a specific driver happens to defer
some events to a thread on a given platform.

Portability note: some drivers defer callbacks
=============================================

While most hardware UART drivers call the async callback from an ISR (UART IRQ,
DMA IRQ, timer IRQ), some drivers intentionally invoke callbacks from a
**workqueue thread** or a dedicated **RX thread**:

- ``drivers/serial/uart_emul.c`` uses a dedicated workqueue to emulate callback
  timing and priority differences (this is explicitly meant to surface
  concurrency bugs).
- ``drivers/serial/uart_native_pty.c`` (native simulator) uses a dedicated RX
  thread and delayed work for some async events.

Portability note: emulation/native drivers may be feature-incomplete
====================================================================

Some non-hardware drivers intentionally trade correctness/completeness for
simplicity. For example, ``uart_native_pty``:

- does not support RX buffer chaining (``rx_buf_rsp`` returns ``-ENOTSUP``),
  and therefore never emits ``UART_RX_BUF_REQUEST``.
- emits ``UART_TX_DONE`` (not ``UART_TX_ABORTED``) from its ``tx_abort`` path,
  with ``len = 0`` to indicate no bytes were actually transmitted.

If your code must run both on real hardware UART drivers and on these simulator
drivers, write it against the API contract, but be prepared to handle
``-ENOTSUP`` and driver-specific limitations when running under emulation.

Even in these cases you should still write your callback as if it was running
in ISR context, because:

- It is still concurrent with application threads.
- It may run at a higher priority than your consumer thread and can preempt it.
- The same code often runs on real hardware drivers where the callback *is* ISR.

That implies:

- No blocking waits (no :c:func:`k_sem_take`, no :c:func:`k_mutex_lock`, no
  sleeping, no long loops).
- Keep the callback bounded and deterministic: do minimal bookkeeping and
  "handoff" work to a thread/workqueue.
- If you share state between callback and threads, you must synchronize:
  :c:func:`k_spin_lock`, or atomics, depending on scope.

Common IRQ-safe "handoff" actions:

- :c:func:`k_sem_give` (ISR → thread wakeup).
- :c:func:`k_work_submit` / :c:func:`k_work_submit_to_queue` (ISR → workqueue).
- :c:func:`k_msgq_put` with ``K_NO_WAIT`` (ISR → message queue).
- Atomic flag set/clear (ISR → thread observation).

What is *not* automatically IRQ-safe:

- Mutating non-atomic data structures (e.g., a ring buffer) without a lock.
- Dynamic allocations, unless the allocator is explicitly IRQ-safe and
  non-blocking for your configuration.
- Heavy parsing and protocol state machines that can run arbitrarily long.


The event model: what drivers may emit
*************************************

The async callback receives a :c:type:`struct uart_event` with these types:

- ``UART_TX_DONE`` / ``UART_TX_ABORTED``
- ``UART_RX_RDY``
- ``UART_RX_BUF_REQUEST`` / ``UART_RX_BUF_RELEASED``
- ``UART_RX_DISABLED`` / ``UART_RX_STOPPED``

The contract in ``uart.h`` defines the intended sequencing and meaning.
Key nuances:

- ``UART_RX_RDY`` may be emitted **multiple times for the same buffer**,
  especially when RX timeouts are enabled (inactivity since last byte).
- ``UART_RX_BUF_REQUEST`` is the driver's way of saying "I am now receiving into
  buffer A; provide buffer B *soon* if you want continuous reception."
- ``UART_RX_BUF_RELEASED`` is the moment you regain ownership of a buffer. You
  must not reuse or free a buffer until you see this event for that buffer.
- ``UART_RX_STOPPED`` means RX stopped due to an error/event (overrun,
  framing, break, etc.). Per the API contract, drivers should then flush any
  pending data via ``UART_RX_RDY``, release buffers, and eventually emit
  ``UART_RX_DISABLED``.

Event sequencing examples (what you should design for)
======================================================

The exact timing depends on the SoC/driver, but the **ordering constraints**
should follow these patterns.

TX
--

- Normal TX:

  - user calls :c:func:`uart_tx`
  - later: ``UART_TX_DONE``

- Aborted TX (explicit abort or timeout under flow control):

  - user calls :c:func:`uart_tx`
  - user calls :c:func:`uart_tx_abort` *or* driver hits a timeout
  - later: ``UART_TX_ABORTED`` (with ``evt->data.tx.len`` = bytes sent)

RX (continuous buffers)
-----------------------

- Continuous RX with timely buffer supply:

  - user calls :c:func:`uart_rx_enable(buf0)` (RX becomes active on ``buf0``)
  - driver emits ``UART_RX_BUF_REQUEST`` (asking for ``buf1``)—**timing varies**
  - user calls :c:func:`uart_rx_buf_rsp(buf1)` (seamless handoff possible)
  - driver emits one or more ``UART_RX_RDY`` as bytes arrive (often on timeout)
  - when ``buf0`` is no longer used: ``UART_RX_BUF_RELEASED(buf0)``
  - driver continues with ``buf1`` and repeats

  **When does ``UART_RX_BUF_REQUEST`` arrive?** This is driver-specific:

  - **nrfx UARTE**: emitted immediately when RX *starts* into a buffer
    ("rxstarted" event), giving maximum time to respond.
  - **STM32, MCUX LPUART**: emitted after :c:func:`uart_rx_enable` returns, in
    the same call context or shortly after from the DMA callback path.
  - **ESP32**: emitted after the initial buffer starts receiving.

  The key invariant is: if you do not supply a buffer via ``uart_rx_buf_rsp``
  before the current buffer fills, RX will stop and ``UART_RX_DISABLED`` will
  be emitted after the release sequence.

- Buffer starvation (no next buffer supplied in time):

  - RX runs on ``buf0``
  - driver emits ``UART_RX_BUF_REQUEST``
  - user does not (or cannot) provide a buffer
  - driver emits ``UART_RX_RDY`` for final data
  - ``UART_RX_BUF_RELEASED(buf0)``
  - ``UART_RX_DISABLED`` (RX session ended; must be re-enabled)

RX (disable / stopped)
----------------------

- Disable initiated by user:

  - user calls :c:func:`uart_rx_disable`
  - driver emits (if needed) a final ``UART_RX_RDY``
  - driver emits ``UART_RX_BUF_RELEASED`` for each scheduled buffer
  - driver emits ``UART_RX_DISABLED`` exactly once

- Stop due to error:

  - driver emits ``UART_RX_STOPPED`` (with stop reason)
  - driver emits (if needed) final ``UART_RX_RDY``
  - releases all buffers (``UART_RX_BUF_RELEASED``)
  - ends with ``UART_RX_DISABLED``

  **Portability note**: Not all drivers emit ``UART_RX_STOPPED``. Some drivers
  (e.g., ESP32) handle errors internally without generating this event, or
  only clear error flags without notifying the user. If your protocol requires
  detecting line errors, verify your target driver's behavior or use
  :c:func:`uart_err_check` as a fallback.


Soundness rules: buffer ownership and lifetimes
***********************************************

These rules are the foundation for both driver implementers and users.

TX buffer ownership
===================

- After a successful :c:func:`uart_tx`, the driver owns the TX buffer contents
  until it emits either ``UART_TX_DONE`` or ``UART_TX_ABORTED``.
- The application must not modify or free that buffer until completion.
- Drivers must not access the buffer after the completion event.

RX buffer ownership
===================

- After a successful :c:func:`uart_rx_enable`, the driver owns the RX buffer
  and will write into it until it emits ``UART_RX_BUF_RELEASED`` for that
  buffer.
- After you provide a second buffer via :c:func:`uart_rx_buf_rsp`, the driver
  owns that buffer until it emits ``UART_RX_BUF_RELEASED`` for it.
- It is **undefined behavior** to provide a buffer that the driver is still
  using (the API states this explicitly).

The result is a strict rule:

- **A buffer becomes reusable only after you observed its
  ``UART_RX_BUF_RELEASED`` event.**


Soundness rules: event and call interference
********************************************

Async UART is deliberately concurrent: the driver can emit events at any time,
including while your application is calling async UART functions.

You must assume:

- Events may arrive immediately after enabling RX/TX.
- Events may arrive while you are in the middle of a state transition in your
  application (e.g., closing a device).
- A call like :c:func:`uart_rx_disable` initiates a *sequence* of events:

  - Potentially a final ``UART_RX_RDY`` (pending data),
  - ``UART_RX_BUF_RELEASED`` for each scheduled buffer,
  - finally ``UART_RX_DISABLED``.

What is definitely not allowed (user side):

- Assuming "disable" means "no more callbacks." You must keep the callback
  installed and able to handle late events until you have observed
  ``UART_RX_DISABLED``.
- Freeing/reusing RX buffers in response to :c:func:`uart_rx_disable` returning.
  You must wait for the release event(s).

What is definitely not allowed (driver side):

- Emitting ``UART_RX_BUF_RELEASED`` for a buffer and then later emitting an
  ``UART_RX_RDY`` that references that buffer again.
- Losing buffers (never releasing them) in error paths; users rely on releases
  for correctness and to avoid leaks.

Time units, "forever" constants, and timeouts
*********************************************

There are two easy-to-miss, but very important details about time arguments in
the async UART API:

- **TX (`uart_tx`) timeout unit**: microseconds.
  The timeout is **only meaningful when hardware flow control is enabled**
  (per the API contract in ``uart.h``). Use ``SYS_FOREVER_US`` to disable the
  timeout. Many drivers simply ignore this timeout when HWFC is off.

- **RX (`uart_rx_enable`) timeout unit**: microseconds.
  The timeout is an *inactivity* period counted from the **last received byte**.
  If no byte has been received yet, no timeout-triggered ``UART_RX_RDY`` event
  will be generated.

Wide-data variants use different units:

- **`uart_tx_u16` timeout unit**: milliseconds (use ``SYS_FOREVER_MS``).
- **`uart_rx_enable_u16` timeout unit**: milliseconds (use ``SYS_FOREVER_MS``).

Detecting async support (user side)
**********************************

The async API does not have a dedicated "supports async" capability query.
The intended probe is:

- Call :c:func:`uart_callback_set`.
  - **`0`**: async callbacks are supported.
  - **`-ENOSYS`**: the device/driver does not implement async callbacks.
  - **`-ENOTSUP`**: async API not enabled in the build/configuration.

Once you have a working callback installed, you can safely use the other async
entry points.

Understanding ``-EACCES`` from ``uart_rx_buf_rsp``
*************************************************

When you call :c:func:`uart_rx_buf_rsp`, you may receive ``-EACCES``. This
means the RX session has already ended (the driver has transitioned to
disabled state) and it is too late to provide a buffer. This commonly occurs
when:

- You deferred buffer provisioning to a thread/workqueue and by the time it
  ran, the current buffer was already full and RX stopped.
- A line error caused ``UART_RX_STOPPED`` → ``UART_RX_DISABLED`` before you
  could respond.

If you receive ``-EACCES``, you should:

- Not treat it as a fatal error.
- Wait for ``UART_RX_DISABLED`` (if not already received).
- Re-enable RX with a fresh buffer via :c:func:`uart_rx_enable`.

Correct interpretation of ``UART_RX_RDY``: always honor (buf, offset, len)
**************************************************************************

When you receive an ``UART_RX_RDY`` event, the bytes that are *newly available*
for processing are in:

.. code-block:: c

   evt->data.rx.buf[evt->data.rx.offset ..
                    evt->data.rx.offset + evt->data.rx.len)

Do not assume ``offset == 0`` and do not assume you will only see one
``UART_RX_RDY`` per buffer. In particular, drivers that implement RX timeouts
will often emit multiple ``UART_RX_RDY`` events for a single buffer as more
bytes arrive.

Code that ignores ``offset`` and treats ``len`` as "total bytes in buffer"
will work on some drivers/configurations but can break on others.


Implementing async UART drivers correctly
****************************************

This section is for driver authors implementing the async hooks in
:c:type:`uart_driver_api` (``callback_set``, ``tx``, ``tx_abort``,
``rx_enable``, ``rx_buf_rsp``, ``rx_disable``).


1) Make the callback contract explicit: ISR context and serialization
=====================================================================

Most existing async drivers invoke the user callback from:

- the UART ISR (e.g., nrfx UARTE error ISR emits ``UART_RX_STOPPED``),
- a DMA ISR/callback (e.g., ESP32 DMA RX completion emits ``RX_RDY`` then
  ``BUF_RELEASED`` then requests next buffer),
- or a timer ISR (e.g., nrfx UARTE RX timeout machinery).

Therefore:

- Treat the callback as **ISR code** inside the driver as well.
- Serialize internal state updates with very short critical sections using
  ``k_spinlock`` (as seen in ns16550/MCUX).

Driver rule of thumb:

- Update internal state *first*, then emit the event.
- Emit events in a way that users can safely react (e.g., if you request a new
  buffer, your ``rx_buf_rsp`` implementation must be safe to call immediately).


2) Implement a clear RX state machine (buffers, offsets, and timeouts)
=====================================================================

The async RX design in Zephyr is a *buffer streaming state machine*:

- One "current" RX buffer and optionally one "next" RX buffer.
- A moving write index in the current buffer.
- A timeout mechanism that converts inactivity into ``UART_RX_RDY`` events
  without ending the overall RX session.

Patterns observed in drivers:

- **DMA counter based** RX (STM32, ns16550, MCUX):
  - Track DMA "bytes received" and compute
    ``len = counter - offset`` for ``UART_RX_RDY``.
  - After emitting ``UART_RX_RDY``, advance ``offset = counter``.
- **DMA completion based** RX (ESP32):
  - When the DMA completes/fires, emit one ``UART_RX_RDY`` for the remaining
    data, reset counters, release the buffer, swap to next, and request another.
- **Early request** behavior (nrfx UARTE):
  - Emit ``UART_RX_BUF_REQUEST`` as soon as RX has started for the current
    buffer ("rxstarted"), to give the user maximum time to respond.

Driver requirements for correctness:

- **Never report stale data**: only emit ``UART_RX_RDY`` for bytes that were not
  already reported for that buffer.
- **Allow multiple ``UART_RX_RDY`` per buffer** (timeout case).
- **Request next buffer early enough** that the user can respond before the
  current buffer becomes full.

Immediate DMA reload in ``rx_buf_rsp``
--------------------------------------

Some DMA-based drivers (e.g., MCUX LPUART) call ``dma_reload()`` immediately
when the user provides the next buffer via ``rx_buf_rsp``. This pre-configures
the DMA controller so that when the current buffer fills, the hardware can
seamlessly switch to the next buffer with minimal software intervention.

This design means:

- The user's response time to ``UART_RX_BUF_REQUEST`` directly affects whether
  seamless reception is possible.
- If the DMA controller supports scatter-gather or linked descriptors, the
  driver can chain buffers without CPU intervention at the switch point.


3) Emit the mandatory teardown sequence on disable/error
========================================================

Users depend on a consistent teardown sequence to reclaim buffers.

On :c:func:`uart_rx_disable`:

- If there is pending data: emit a final ``UART_RX_RDY``.
- For every scheduled buffer (current and next): emit ``UART_RX_BUF_RELEASED``.
- Emit ``UART_RX_DISABLED`` exactly once.

On RX error/stop (``UART_RX_STOPPED``):

- Emit ``UART_RX_STOPPED`` with a correct :c:type:`uart_rx_stop_reason`.
- Then follow the same rules as disable: flush data, release buffers, end with
  ``UART_RX_DISABLED``.

The nrfx UARTE driver is a good example of emitting ``UART_RX_STOPPED`` from an
error ISR, then shutting down RX.


4) Define and enforce reentrancy rules for API calls from callbacks
===================================================================

Users frequently call some UART async APIs from inside the callback.
Examples in-tree:

- Supplying buffers via :c:func:`uart_rx_buf_rsp` in ``UART_RX_BUF_REQUEST``.
- Re-enabling RX in response to ``UART_RX_DISABLED`` (common recovery pattern).
- Issuing a new TX from ``UART_TX_DONE`` (sample/queue patterns).

As a driver author, you should support at least:

- Calling ``rx_buf_rsp`` from callback context (this is the intended design).
- Calling ``rx_enable`` from ``UART_RX_DISABLED`` callback context for recovery.

If your implementation cannot support a call safely in ISR context, you must:

- document it clearly (Kconfig help and driver docs), and/or
- return a deterministic error code, not deadlock.

Avoid implicit recursion hazards:

- Do not hold a driver spinlock while invoking the user callback if your API
  entry points take the same lock. Either:
  - release the lock before calling the user, or
  - use a lockless "event queue" and invoke callback after releasing locks.

Implementation pattern for ``rx_buf_rsp`` reentrancy (observed in-tree):

Most drivers protect ``rx_buf_rsp`` with a spinlock to prevent races with
ISR-driven buffer swaps. The typical pattern is:

- Acquire a spinlock.
- Check if a next buffer is already set (return ``-EBUSY`` if so).
- Check if RX is still enabled (return ``-EACCES`` if not—too late).
- Store the buffer pointer and length.
- For DMA drivers: call ``dma_reload()`` to pre-configure the next transfer.
- Release lock.

This pattern ensures that the buffer is safely registered before the current
buffer finishes, enabling seamless double-buffering.


5) Handle DMA, caches, and memory placement explicitly
======================================================

Many async UART drivers use DMA and therefore must handle:

- Buffer alignment and addressability constraints.
- Data cache coherency (invalidate/clean) or "nocache" memory regions.

The UART async test suite explicitly supports nocache configurations and
expects drivers to handle RX buffers correctly under cache.

Driver guidance:

- Validate DMA constraints in ``rx_enable``/``tx``:
  - return ``-EINVAL`` / ``-ENOTSUP`` for invalid buffers,
  - or perform required cache maintenance.
- Make the behavior deterministic. Silent data corruption is worse than
  returning an error.

Cyclic (circular) DMA mode
--------------------------

Some drivers (e.g., STM32, MCUX LPUART) support a cyclic DMA mode where a
single buffer is used continuously without requiring buffer swaps. In this
mode:

- ``UART_RX_BUF_REQUEST`` may not be emitted, or is handled differently.
- ``UART_RX_RDY`` is emitted at half-complete and full-complete points.
- The same buffer is reused automatically.

This mode is typically configured via Devicetree or Kconfig, not the API. If
your driver supports cyclic mode, document clearly how the event sequence
differs and whether users need to respond to ``UART_RX_BUF_REQUEST``.


6) Power management and runtime PM
==================================

Async DMA transfers may outlive the calling thread.

If your driver supports PM/runtime PM:

- Keep the UART clock running while a transfer is active.
- Release the PM constraint *after* emitting the completion event.

Some drivers must avoid blocking in ISR context to decide when the hardware is
fully idle (e.g., "TX FIFO empty and shift register empty"). In those cases,
offload "wait-until-idle then unlock PM" to a workqueue rather than spinning or
sleeping in interrupt context.

The uart_async_dual tests verify runtime PM state transitions during active
transfers.


Using async UART drivers correctly
**********************************

This section is for subsystem/app authors using :c:func:`uart_tx`,
:c:func:`uart_rx_enable`, :c:func:`uart_rx_buf_rsp`, etc.


1) Treat the callback as an ISR: keep it tiny and hand off
=========================================================

Recommended structure:

- Callback (ISR):
  - update a small amount of state (atomics),
  - copy a small chunk into a staging buffer or ring buffer (optionally),
  - signal a thread/work item.
- Thread/work handler:
  - do parsing, allocation, protocol logic, logging,
  - call the rest of your subsystem.

Use one of these proven patterns:

- Semaphore handoff (callback posts a semaphore; thread drains/handles).
- Workqueue handoff (callback submits work; work handler does heavy lifting).
- Message queue handoff (callback posts bounded messages with ``K_NO_WAIT``).

If you are throughput-bound, also consider the patterns for:

- **High-throughput ring buffers with claim/finish** (minimize copies and
  contention; pair well with ``UART_RX_RDY`` bursts).
- **Zero-copy packet buffers (SPSC/MPSC pbuf)** when you want explicit overflow
  policy (drop vs overwrite) and strict ownership semantics between producers
  (ISR/callback) and a single consumer thread.


2) Know which calls are "expected" from callback context
========================================================

Safe and intended from the callback:

- ``uart_rx_buf_rsp()`` in response to ``UART_RX_BUF_REQUEST``.

Conditionally acceptable (driver-dependent, but used in-tree):

- ``uart_rx_enable()`` in response to ``UART_RX_DISABLED`` (recovery).
- ``uart_tx()`` in response to ``UART_TX_DONE`` (TX queueing).
- ``uart_rx_disable()`` / ``uart_tx_abort()`` as part of an error policy.

The safest rule if you want portable behavior across drivers:

- **In the callback, only call ``uart_rx_buf_rsp()`` and IRQ-safe kernel APIs.**
  Defer everything else to a thread/workqueue.

One more nuance: because the callback is invoked by the driver, calling back
into the same driver can create lock recursion or state-machine reentrancy.
Even if a call "works" on one driver, it may deadlock or fail on another unless
the driver explicitly designed for it.


3) Continuous RX: always be ready to answer ``UART_RX_BUF_REQUEST``
===================================================================

If you ignore ``UART_RX_BUF_REQUEST``:

- RX will stop when the current buffer fills, and you will get
  ``UART_RX_DISABLED`` (after releases).

To stream continuously:

- Maintain a pool of RX buffers and respond promptly with ``uart_rx_buf_rsp``.

Two common approaches:

- **Fixed double-buffer** (simple, used in samples/drivers):
  - have two statically allocated buffers,
  - on ``RX_BUF_REQUEST``, provide the other,
  - on ``RX_BUF_RELEASED``, mark it as available.
- **Buffer pool + reference counting** (more robust, used in modem backends):
  - allocate buffers from :c:func:`k_mem_slab_alloc` with ``K_NO_WAIT`` in ISR,
  - free them on ``RX_BUF_RELEASED``,
  - if you need to keep data beyond the callback, take a ref and release later.

Zephyr also provides ``uart_async_rx`` helper for users that want a zero-copy,
multi-buffer RX stream with safe claim/consume semantics.

Handling ``uart_rx_buf_rsp`` errors in the callback
---------------------------------------------------

When calling ``uart_rx_buf_rsp`` from the ``UART_RX_BUF_REQUEST`` callback,
check the return value:

- **0**: Buffer successfully registered.
- **-EBUSY**: A next buffer is already set (should not happen if you only call
  once per request).
- **-EACCES**: RX is already disabled (too late). Release the buffer back to
  your pool; you will receive ``UART_RX_DISABLED`` shortly.
- **-ENOTSUP**: Driver does not support buffer chaining (e.g., native_pty).

Example (from shell UART async backend):

.. code-block:: c

   case UART_RX_BUF_REQUEST:
       buf = uart_async_rx_buf_req(&sh_uart->async_rx);
       if (buf) {
           int err = uart_rx_buf_rsp(dev, buf, len);
           if (err < 0) {
               /* Return buffer to pool on error */
               uart_async_rx_on_buf_rel(&sh_uart->async_rx, buf);
           }
       }
       break;

Avoid a common pitfall: "blindly cycling" RX buffers
----------------------------------------------------

Some code chooses to ignore ``UART_RX_BUF_RELEASED`` and simply cycles through
a fixed set of RX buffers when ``UART_RX_BUF_REQUEST`` arrives. This can appear
to work if the consumer is always fast enough, but it relies on a timing
assumption: that a buffer will always be released by the time you reuse it.

If that assumption is ever violated (higher baud rate, longer ISR latency, or a
burst), you can accidentally provide a buffer that is still in use, which the
API defines as **undefined behavior**.

If you want correctness under load, track releases (or use ``uart_async_rx``)
so that you only reuse buffers that were actually released.

Pattern: atomic flags for buffer tracking
-----------------------------------------

The modem backend demonstrates a robust pattern using atomic flags:

.. code-block:: c

   /* State bits */
   #define RX_BUF0_USED_BIT  0
   #define RX_BUF1_USED_BIT  1

   /* In UART_RX_BUF_REQUEST handler */
   if (!atomic_test_and_set_bit(&state, RX_BUF0_USED_BIT)) {
       uart_rx_buf_rsp(dev, buf0, len);
   } else if (!atomic_test_and_set_bit(&state, RX_BUF1_USED_BIT)) {
       uart_rx_buf_rsp(dev, buf1, len);
   } else {
       /* Both buffers in use - cannot provide */
   }

   /* In UART_RX_BUF_RELEASED handler */
   if (evt->data.rx_buf.buf == buf0) {
       atomic_clear_bit(&state, RX_BUF0_USED_BIT);
   } else if (evt->data.rx_buf.buf == buf1) {
       atomic_clear_bit(&state, RX_BUF1_USED_BIT);
   }

This pattern guarantees you never provide a buffer that hasn't been released,
even under race conditions.


4) ``UART_RX_RDY`` is incremental: respect (buf, offset, len)
=============================================================

The RX event indicates that new data is in:

.. code-block:: c

   evt->data.rx.buf[evt->data.rx.offset ..
                    evt->data.rx.offset + evt->data.rx.len)

Do not assume:

- The event covers the entire buffer.
- The offset is zero.
- You will get exactly one ``RX_RDY`` per buffer.

Sound RX handling patterns:

- Copy data out (bounded copy) and signal a thread.
- Or, with ``uart_async_rx`` helper:
  - call ``uart_async_rx_on_rdy()`` in the callback,
  - consume data from thread via ``uart_async_rx_data_claim()`` /
    ``uart_async_rx_data_consume()``.


5) Protect shared queues/ring buffers (do not assume they are ISR-safe)
======================================================================

If the callback pushes bytes into a ring buffer and a thread consumes it, you
must protect ring buffer operations with a lock (spinlock), unless
you are using a data structure with explicit lock-free guarantees for that
producer/consumer topology.

Good patterns seen in-tree:

- Use ``k_spinlock`` around ring buffer operations (modem backend).
- Use ``uart_async_rx`` helper which internalizes atomic bookkeeping (shell).
- Use ``k_msgq_put(..., K_NO_WAIT)`` to pass (pointer,len) events to a thread,
  and only copy in thread context (modem_backend_uart_async_hwfc).

Throughput-focused options (from the concurrency pattern set)
------------------------------------------------------------

If your RX path must sustain high rates with low overhead, prefer designs that
make ownership and commit points explicit:

- **Ring buffer claim/finish**:
  - Callback reserves space using ``ring_buf_put_claim()``, writes/copies into
    the claimed region, then commits via ``ring_buf_put_finish()``.
  - Consumer drains using ``ring_buf_get_claim()`` / ``ring_buf_get_finish()``.
  - Apply external locking if you have multiple producers.

- **SPSC/MPSC packet buffers**:
  - Use SPSC when exactly one producer (the UART callback) feeds one consumer.
  - Use MPSC if multiple producers feed one consumer and you need a defined
    overflow policy (drop vs overwrite) and drop accounting.

6) Backpressure: when you cannot keep up, disable RX deliberately
===============================================================

If your RX consumer can fall behind (e.g., protocol parsing, logging, network
stack backpressure), you have two choices:

- **Provision enough buffering** (more/larger RX buffers, larger ring buffer),
  and accept latency.
- **Apply backpressure** and accept gaps by stopping RX temporarily.

An in-tree example is the PPP UART async backend: when the RX ring buffer is
close to full, it calls :c:func:`uart_rx_disable` from within ``UART_RX_RDY`` to
avoid overruns, and then re-enables RX later after ``UART_RX_DISABLED`` via
delayed work.

Guidance:

- In the callback, prefer setting an atomic "need_rx_pause" flag and deferring
  :c:func:`uart_rx_disable` to a thread/workqueue unless you have validated the
  driver tolerates it from callback context.
- Always design for the full disable sequence: a final ``UART_RX_RDY`` may still
  occur, then releases, then ``UART_RX_DISABLED``.
- Make sure your RX buffer pool logic can handle "RX stops often" (no leaks; no
  double-free; no reuse-before-release).

If you choose "drop instead of pause", prefer explicit drop accounting:

- Track dropped bytes/frames in atomics.
- Use a data structure with explicit drop semantics (e.g., MPSC packet buffer
  configured to drop-on-full) rather than silently overwriting or corrupting.

7) TX serialization: expect ``-EBUSY`` and build a queue
========================================================

Only one TX can be active per device. If you call :c:func:`uart_tx` while a TX
is in progress, you will typically get ``-EBUSY``.

Robust TX patterns:

- **Semaphore "blocking TX" wrapper** (shell UART async write):
  - start ``uart_tx``,
  - wait for ``UART_TX_DONE`` by taking a semaphore in thread context.
- **TX queue**:
  - if ``uart_tx`` returns ``-EBUSY``, enqueue the buffer,
  - on ``UART_TX_DONE``, dequeue and start the next TX.

Avoid calling :c:func:`uart_tx` from multiple threads without a higher-level
serializer; you will otherwise create spurious ``-EBUSY`` and reordering.

Scheduling note (worker threads)
--------------------------------

If you drain a RX FIFO/msgq or process large bursts, consider occasionally
calling :c:func:`k_yield()` in the worker thread loop (not in the UART callback)
to avoid starving peer threads at the same priority, as described in the
"thread priorities, yielding, and sleeping" pattern.


8) Recovery: handle ``UART_RX_STOPPED`` and ``UART_RX_DISABLED``
===============================================================

For line errors and noise you should define an explicit policy:

- On ``UART_RX_STOPPED``:
  - record the error reason,
  - expect that RX will end and buffers will be released.
- On ``UART_RX_DISABLED``:
  - decide whether to re-enable RX (recovery) or leave it disabled.

In-tree recovery styles:

- Immediate re-enable in callback (modem iface; tests).
- Deferred re-enable via scheduled work (recommended for portability).


9) Do not mix async with polling/interrupt APIs on the same UART instance
=========================================================================

Mixing APIs causes callback interference and undefined behavior.
Concrete conflicts:

- Polling RX functions return ``-EBUSY`` when async RX is enabled.
- Interrupt-driven and async both require ownership of UART IRQ callbacks.

If you need interrupt-driven semantics on top of an async-only UART, consider
using the adapter in ``drivers/serial/uart_async_to_irq.c`` (it uses
``uart_async_rx`` to safely bridge RX into FIFO-like reads).


Checklist (quick)
*****************

Implementers (drivers)
======================

- ``callback_set`` stores handler + user_data and enforces exclusive callbacks.
- All async API entry points are deterministic:

  - ``-EBUSY`` for active transfers or next buffer already set.
  - ``-EFAULT`` for "nothing to abort/disable".
  - ``-EACCES`` for too-late ``rx_buf_rsp`` (RX already disabled).

- Every RX buffer provided is eventually released with ``UART_RX_BUF_RELEASED``.
- ``UART_RX_DISABLED`` is emitted exactly once per RX session end.
- Correct handling for repeated ``UART_RX_RDY`` events and offsets.
- DMA + cache coherency is handled explicitly.
- ``rx_buf_rsp`` is safe to call from callback context (uses spinlock).

Users (subsystems/apps)
=======================

- Callback is ISR: bounded, non-blocking, minimal work.
- You only reuse buffers after ``UART_RX_BUF_RELEASED``.
- You handle ``-EBUSY`` from ``uart_tx`` and serialize TX.
- RX buffer pool never starves (or you accept RX stopping by design).
- You protect shared data structures with the right primitive.

Testing tip
===========

If you are building a new async-UART-based protocol, run it under intentional
contention and jitter (busy/idle injection, randomized timing, high-rate bursts)
to shake out races and buffer ownership bugs. In-tree references that
specifically exercise async UART under stress are:

- ``tests/drivers/uart/uart_async_rx`` (uses ``ztress``).
- ``tests/drivers/uart/uart_async_dual`` (uses ``busy_sim`` and includes
  ``pm_runtime`` cases).


