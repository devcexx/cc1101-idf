# Transmit Sync Mode

This example transmits a continuous signal oscillating at 16 kHz using
the synchronous transmission mode of the CC1101. This example uses
interrupts to transmit each change in the signal at the raising edge
of each clock pulse emitted by the CC1101 through the GPO2 pin.
