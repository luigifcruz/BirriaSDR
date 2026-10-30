/*
MIT License

Copyright (c) 2026 Luigi C.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
.equ MAILBOX, 0x2038360
.equ BOOT_MAGIC, 0x41495242
.equ RESPONSE_XOR, 0xb1771a5d
.equ RING, 0x2040000
.equ HEADERS, 0x2040100
.equ PAYLOAD, 0x2041000
.equ SLOT_STRIDE, 0x3000
.equ RING_COUNT, 0x4
.equ PAYLOAD_BYTES, 0x2000
/*
 * Original continuous I/Q exporter from capture SRAM to serial USB DMA.
 * The only ROM call powers on the PHY. No vendor RAM firmware is downloaded.
 * The immutable boot ROM handles USB enumeration and program download.
 * The host owns RF tuning and gain, then releases GO after queuing USB reads.
 * All descriptors and headers reside in dedicated MCU data memory.
 * The program parks after publishing a terminal state until a subsystem reset.
 * Opted-in continuous reads instead recover source faults inside this program.
 *
 * Mailbox offsets are bytes from MAILBOX. The host supplies the nonce at +4,
 * packet limit at +32 (zero means until STOP), STOP at +36 and GO at +60.
 * The program reports state at +28, submitted/completed counts at +40/+44,
 * elapsed microseconds at +48 and DMA error status at +52.
 * The exporter submits one SRAM descriptor at a time and waits for completion
 * before releasing its source bytes. It tracks producer progress while waiting
 * for both data and DMA, reporting source overrun or ambiguous producer timing
 * as explicit gaps or terminal failures rather than hiding overwritten samples.
 *
 * State values: 1 ready/running, 2 finite done, 3 host stop, 4 DMA timeout,
 * 5 GO timeout, 6 initial DMA busy, 8 ambiguous wrap, 9 source overrun and
 * 10 source timeout. Source faults and normal stops drain the descriptor before
 * stopping capture.
 * Extra mailbox offsets: +64 produced units, +68 final capture control,
 * +72 producer position, +76 ROM power-on result, +80 maximum backlog and
 * +84 maximum poll interval in microseconds. The scratch words at +88/+92
 * hold the preceding poll time and completed source units.
 * The host enables local recovery at +96. The words at +100/+104 hold the
 * packet count at the current SRAM epoch and the cumulative gap count.
 * The scratch flag at +108 records whether ICAP is actively producing.
 * A 48-byte event (extended ID 0xf6) precedes each new epoch. It carries the
 * fault at header +8, gap count at +12, next data sequence at +36 and nonce
 * at +40. Its four-byte trailer is not data. The host must discard the last
 * held data packet because DMA may have read it while ICAP overwrote it.
 * Recovery drains DMA, stops ICAP, sends and drains this marker, then restarts
 * ICAP at offset zero. It does not reset USB, the PHY, gain or the synthesizer.
 * Only known source faults are recoverable. DMA errors and timeouts still park.
 */
    .section .text.entry,"ax"
    .global _start
    .type _start, @function
    .macro load32 reg,value
    sethi \reg, (\value >> 12)
    ori \reg, \reg, (\value & 0xfff)
    .endm
    /* Observe the live capture even while USB is stalled. One address unit is
     * four SRAM bytes: one complex sample at 40 and 80 MSps, or one of the
     * four words in a packed record at 10 and 20 MSps, where each record holds
     * two samples and two zero words. The ring contains 65536 units, which
     * last 1.6384 ms at 20 and 40 MSps and 0.8192 ms at 80 MSps. The 750 us
     * poll guard, still shorter than the 80 MSps ring, rejects ambiguous
     * producer movement before a whole ring can pass unnoticed.
     * Scratch r0/r1/r3, preserve the DMA wait counter (r2) and ring registers. */
    .macro observe_source
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 88]
    swi $r0, [$fp + 88]
    sub $r1, $r0, $r1
    lwi $r3, [$fp + 84]
    slt $r3, $r3, $r1
    beqz $r3, 90f
    swi $r1, [$fp + 84]
90:
    slti $r3, $r1, 750
    beqz $r3, timing_fault
    load32 $r0, 0x800210b4
    lwi $r0, [$r0]
    zeh33 $r0, $r0
    lwi $r1, [$fp + 72]
    swi $r0, [$fp + 72]
    sub $r1, $r0, $r1
    zeh33 $r1, $r1
    lwi $r0, [$fp + 64]
    add $r0, $r0, $r1
    swi $r0, [$fp + 64]
    lwi $r1, [$fp + 92]
    sub $r0, $r0, $r1
    lwi $r3, [$fp + 80]
    slt $r3, $r3, $r0
    beqz $r3, 91f
    swi $r0, [$fp + 80]
91:
    movi $r3, 61440
    slt $r3, $r0, $r3
    beqz $r3, source_overrun
    .endm
_start:
    load32 $fp, MAILBOX
    mfsr $r0, $psw
    swi $r0, [$fp + 12]
    swi $sp, [$fp + 16]
    swi $gp, [$fp + 20]
    swi $lp, [$fp + 24]
    lwi $r0, [$fp + 4]
    load32 $r1, RESPONSE_XOR
    xor $r0, $r0, $r1
    swi $r0, [$fp + 8]
    /* Use the immutable ROM only for PHY power-on. */
    load32 $r2, 0x00837142
    jral5 $r2
    swi $r0, [$fp + 76]
    load32 $r6, 0x54000300
    load32 $r7, RING
    /* ROM handoff disables both directions. Only take an idle engine. */
    lwi $r0, [$r6 - 248]
    andi $r1, $r0, 10
    bnez $r1, initial_busy
    bitci $r0, $r0, 5
    bclr $r0, $r0, 30
    swi $r0, [$r6 - 248]
    movi $r0, 0
    swi $r0, [$r6 - 252]  /* 0x54000204: mask DMA interrupts */
    /* Clear all our ring slots and 64-byte event headers. */
    load32 $r1, RING
    movi $r2, (RING_COUNT * 4)
clear_ring:
    swi.bi $r0, [$r1], 4
    addi $r2, $r2, -1
    bnez $r2, clear_ring
    load32 $r1, HEADERS
    movi $r2, (RING_COUNT * 16)
clear_headers:
    swi.bi $r0, [$r1], 4
    addi $r2, $r2, -1
    bnez $r2, clear_headers
    /* Initialize each event header with this acquisition's nonce. */
    load32 $r8, HEADERS
    load32 $r9, PAYLOAD
    movi $r10, RING_COUNT
initialize_slot:
    load32 $r0, (0x3800002c + PAYLOAD_BYTES)
    swi $r0, [$r8 + 0]
    load32 $r0, (0x38000014 + PAYLOAD_BYTES)
    swi $r0, [$r8 + 24]
    movi $r0, 0xed
    swi $r0, [$r8 + 28]
    movi $r0, 0xf5
    swi $r0, [$r8 + 32]
    lwi $r0, [$fp + 4]
    swi $r0, [$r8 + 40]
    addi $r8, $r8, 64
    addi $r9, $r9, SLOT_STRIDE
    addi $r10, $r10, -1
    bnez $r10, initialize_slot
    dsb
    swi $r7, [$r6 + 0]
    movi $r0, RING_COUNT
    swi $r0, [$r6 + 4]
    movi $r0, 1
    swi $r0, [$r6 - 244] /* 0x5400020c: reset TX0 DMA index */
    movi $r0, 0
    swi $r0, [$r6 + 8]
    lwi $r0, [$r6 - 248]
    ori $r0, $r0, 1      /* TX enabled; RX remains disabled */
    swi $r0, [$r6 - 248]
    movi $r0, 1
    swi $r0, [$fp + 28]
    dsb
    load32 $r0, BOOT_MAGIC
    swi $r0, [$fp]
    dsb
    movi $r4, 0          /* sequence */
    movi $r5, 0          /* producer index */
    swi $r4, [$fp + 108]
    load32 $r2, 100000000
wait_go:
    lwi $r0, [$fp + 60]
    bnez $r0, begin_stream
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    addi $r2, $r2, -1
    bnez $r2, wait_go
    movi $r0, 5
    j finish
begin_stream:
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + 56]
    movi $r1, 0
    swi $r1, [$fp + 104]
    swi $r1, [$fp + 80]
    swi $r1, [$fp + 84]
restart_capture:
    /* Reset only ICAP's capture cursor, after all DMA readers have drained. */
    load32 $r0, 0x80021090
    movi $r1, 0x4f1
    swi $r1, [$r0]
    dsb
    swi $r4, [$fp + 100]
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + 88]
    movi $r0, 0
    swi $r0, [$fp + 64]  /* produced address units, modulo 2^32 */
    swi $r0, [$fp + 72]  /* last 16-bit producer index */
    swi $r0, [$fp + 92]  /* completed address units, modulo 2^32 */
    load32 $r0, 0x80021090
    movi $r1, 0x24f3
    movi $r3, 1
    swi $r3, [$fp + 108]
    swi $r1, [$r0]
publish:
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    lwi $r0, [$fp + 32]
    beqz $r0, next_packet
    beq $r4, $r0, done
next_packet:
    /* Wait for the preceding SRAM descriptor before publishing the next one. */
    addi $r10, $r5, 1
    andi $r10, $r10, (RING_COUNT - 1)
    load32 $r2, 100000000
wait_space:
    lwi $r0, [$r6 + 12]
    sub $r1, $r5, $r0
    andi $r1, $r1, (RING_COUNT - 1)
    sub $r1, $r4, $r1
    swi $r1, [$fp + 44]
    lwi $r0, [$fp + 100]
    sub $r1, $r1, $r0
    slli $r1, $r1, 11
    swi $r1, [$fp + 92]
    observe_source
    lwi $r0, [$r6 + 12]
    /* SRAM reads require serial descriptors on this part. Multiple queued
     * SRAM packets have reproduced silent payload corruption. */
    beq $r0, $r5, space_ready
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    addi $r2, $r2, -1
    bnez $r2, wait_space
    movi $r0, 4
    j finish
space_ready:
    lwi $r1, [$fp + 100]
    sub $r1, $r4, $r1
    slli $r1, $r1, 11
    swi $r1, [$fp + 92]
    load32 $r2, 1000000
wait_source:
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    observe_source
    slti $r3, $r0, (PAYLOAD_BYTES / 4)
    beqz $r3, source_ready
    addi $r2, $r2, -1
    bnez $r2, wait_source
    movi $r10, 10
    j drain
source_ready:
    slli $r0, $r5, 6
    load32 $r8, HEADERS
    add $r8, $r8, $r0
    /* A slot previously used for a gap must become an ordinary data header. */
    load32 $r0, (0x3800002c + PAYLOAD_BYTES)
    swi $r0, [$r8]
    load32 $r0, (0x38000014 + PAYLOAD_BYTES)
    swi $r0, [$r8 + 24]
    movi $r0, 0xf5
    swi $r0, [$r8 + 32]
    movi $r0, 0
    swi $r0, [$r8 + 8]
    swi $r0, [$r8 + 12]
    /* Direct DMA from SRAM: no CPU copying or sample conversion.
     * USB consumes four extra trailer bytes, discarded by the host. The
     * last packet reads these from the adjacent, mapped SRAM word. */
    lwi $r0, [$fp + 100]
    sub $r0, $r4, $r0
    andi $r0, $r0, 31
    slli $r0, $r0, 13
    load32 $r9, 0xe0200000
    add $r9, $r9, $r0
    swi $r4, [$r8 + 36]
    slli $r0, $r5, 4
    add $r0, $r7, $r0
    swi $r8, [$r0]
    swi $r9, [$r0 + 8]
    movi $r1, 0
    swi $r1, [$r0 + 12]
    load32 $r1, ((44 << 16) | (1 << 14) | (PAYLOAD_BYTES + 4))
    swi $r1, [$r0 + 4]
    dsb
    addi $r5, $r10, 0
    swi $r5, [$r6 + 8]
    addi $r4, $r4, 1
    swi $r4, [$fp + 40]
    j publish
done:
    movi $r10, 2
    j drain
stopped:
    movi $r10, 3
drain:
    /* Drain before disabling SRAM. The final source packet is untrusted after
     * a source fault, so the ordered marker tells the host to discard it. */
    load32 $r2, 100000000
wait_dma:
    /* A normal STOP must still detect overwrite of its final descriptor. */
    lwi $r0, [$fp + 108]
    beqz $r0, drain_fault
    slti $r0, $r10, 8
    beqz $r0, drain_fault
    observe_source
drain_fault:
    lwi $r0, [$r6 + 12]
    beq $r0, $r5, completed
    addi $r2, $r2, -1
    bnez $r2, wait_dma
    movi $r0, 4
    j finish
completed:
    swi $r4, [$fp + 44]
    slti $r0, $r10, 8
    bnez $r0, terminal
    slti $r0, $r10, 10
    beqz $r0, terminal
    lwi $r0, [$fp + 96]
    beqz $r0, terminal
    lwi $r0, [$fp + 32]
    bnez $r0, terminal
    lwi $r0, [$r6 - 496]
    bnez $r0, terminal
    j recover_source
terminal:
    addi $r0, $r10, 0
    j finish
recover_source:
    /* The RF chain stays configured. Stop only the SRAM writer, and keep it
     * stopped until the ordered gap marker has left the device DMA engine. */
    load32 $r0, 0x80021090
    movi $r1, 0x4f0
    swi $r1, [$r0]
    movi $r1, 0
    swi $r1, [$fp + 108]
    dsb
    slli $r0, $r5, 6
    load32 $r8, HEADERS
    add $r8, $r8, $r0
    load32 $r0, 0x3800002c
    swi $r0, [$r8]
    swi $r10, [$r8 + 8]
    lwi $r0, [$fp + 104]
    addi $r0, $r0, 1
    swi $r0, [$fp + 104]
    swi $r0, [$r8 + 12]
    load32 $r0, 0x38000014
    swi $r0, [$r8 + 24]
    movi $r0, 0xf6
    swi $r0, [$r8 + 32]
    swi $r4, [$r8 + 36]
    /* Both marker fragments are in MCU data RAM, not the stopped SRAM path. */
    slli $r0, $r5, 4
    add $r0, $r7, $r0
    swi $r8, [$r0]
    addi $r1, $r8, 44
    swi $r1, [$r0 + 8]
    movi $r1, 0
    swi $r1, [$r0 + 12]
    load32 $r1, ((44 << 16) | (1 << 14) | 4)
    swi $r1, [$r0 + 4]
    dsb
    addi $r5, $r5, 1
    andi $r5, $r5, (RING_COUNT - 1)
    swi $r5, [$r6 + 8]
    load32 $r2, 100000000
wait_marker:
    lwi $r0, [$r6 + 12]
    beq $r0, $r5, marker_complete
    addi $r2, $r2, -1
    bnez $r2, wait_marker
    movi $r0, 4
    j finish
marker_complete:
    lwi $r0, [$r6 - 496]
    bnez $r0, terminal
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    j restart_capture
initial_busy:
    movi $r0, 6
finish:
    addi $r10, $r0, 0
    load32 $r0, 0x80021090
    movi $r1, 0x4f0
    swi $r1, [$r0]
    lwi $r1, [$r0]
    swi $r1, [$fp + 68]
    lwi $r1, [$r0 + 36]
    lwi $r2, [$fp + 72]
    swi $r1, [$fp + 72]
    sub $r1, $r1, $r2
    zeh33 $r1, $r1
    lwi $r2, [$fp + 64]
    add $r1, $r1, $r2
    swi $r1, [$fp + 64]
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 56]
    sub $r0, $r0, $r1
    swi $r0, [$fp + 48]
    lwi $r0, [$r6 - 496] /* 0x54000110: DMA errors */
    swi $r0, [$fp + 52]
    dsb
    swi $r10, [$fp + 28] /* Publish terminal state after all result fields. */
    dsb
park:
    j park
timing_fault:
    movi $r10, 8
    j drain
source_overrun:
    movi $r10, 9
    j drain
    .size _start, .-_start

