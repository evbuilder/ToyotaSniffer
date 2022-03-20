# ToyotaSniffer

Toyota has devised a comms link between Hybrid ECU and motor controllers that is a mashup of synchronous serial and CANbus.

Others have paved the way with the Lexus GS450. See https://openinverter.org/wiki/ToyotaInverters
The openinverter forum has more information, but not obviously connected to the above link.
Github user @damienmaguire has some interesting data dumps, and very cool hardware to control the Lexus and Auris/Yaris hybrids. (in progress)
https://github.com/damienmaguire/Prius-Gen3-Inverter/blob/master/Serial_Logs/yaris1_logicboard_onown.logicdata

On the other side of the world, we have Aqua and Prius C hybrids. I'm building this sniffer to see if the locally available models behave the same or different. Plan is to make that data available.

Protocol oddities:
---
- 4 pairs of CAN level signals: htm, mth, req, clk.
- htm and mth are serial data framed as if asynchronous (start and stop bit) but are aligned with clk.
- req seems to be an interupt signal
- Transactions occur at 200Hz
- at least 120 bytes transferred >1200bits.
-  gs450H_v7.ino appears set for 250kbaud, however measures 500kbaud.

Sniffer design
---
Because of the async framing bits, we don't actually need the clk signal.
The ESP32-S2 has a configurable receive timeout interrupt, which means we don't need the req signal either.
Expected output rate is (120+100 Bytes) * 200Hz = 44kBps.
For this commmit, the data buffers are serviced by one task which sends out complete messages in binary with a minimal prefix. This seems to send out all data - with 314 kBytes sent in ~5 seconds.
