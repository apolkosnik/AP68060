# AP68060 in Minimig (MiSTer)

`ap060_tg68k_compat.v` puts the AP68060 where Minimig's `cpu_wrapper.v`
instantiates TG68K (`TG68KdotC_Kernel`): it presents the same ports
(`clkena_in`, `busstate`, 16-bit `data_in`/`data_write`, `nuds`/`nlds`,
`longword`, `cacr_out`, `vbr_out`, ...) and runs the 68060 bus behind them.

- **Clocks.** The processor runs on its own clock (`clk_cpu`, e.g. 55 MHz
  from a separate PLL), unrelated to `cpu_wrapper`'s. The two halves cross
  with request/answer toggles through synchronisers; IPL, reset, RSTO, VBR
  and the Fast RAM windows have synchronisers of their own. Declare the
  clocks asynchronous in the SDC.
- **Bus.** Each 68060 transfer outside Fast RAM becomes TG68K-style 16-bit
  accesses (a long word is two, `longword` high on the first, as Gayle's
  32-bit IDE port expects, with the sampled idle clock `cpu_wrapper` wants
  between long words). A line is one request: its eight words run on the
  `clk` side and its four beats return on consecutive clocks, without TBI.
  Writes are posted through two request slots, so the processor goes on
  while the `clk` side completes them; a read waits for its own answer,
  which follows every write queued before it. Reads outside Fast RAM and
  Kickstart ROM get TCI: chip RAM, I/O and the RTG frame buffer are never
  cached.
- **Fast RAM.** The Zorro II/III windows never go through `cpu_wrapper`:
  `rtl/fastram/ap060_fastram` answers them on the 68060 bus and reaches
  DDR3 as an Avalon-MM master on `clk_mem` (ports `clk_mem`, `fr_*`).
  `cpu_wrapper` passes those ports through, and `Minimig.sv` puts an
  `a2065_ddram_arbiter` in front of DDRAM with this port first and
  `ddram_ctrl` second. The DDR3 word address follows Minimig's
  `memory_router` mapping of Zorro RAM (Zorro II at `ramaddr[28:27] = 10`),
  so the 16-bit path and this port see the same memory. The NMI vector read
  (VBR + $7C) stays on the Minimig bus, where the HRTmon cartridge
  overlays it.
- **Interrupts.** An interrupt acknowledge first runs TG68K's acknowledge
  read on the Minimig bus (`$FFFFFFF0 + level * 2`), then ends in TA with
  AVEC. The Action Replay / HRTmon cartridge (`cart.v`) arms on level 7's
  read and drops its NMI request with it; answering the acknowledge inside
  the adapter leaves HRTmon unreachable and IPL stuck at 7.
- **Memory-controller caches.** `cacr_out`/`d_cache_out` keep Minimig's
  controller caches (`cpu_cache_new` in `sdram_ctrl`/`ddram_ctrl`) always
  on. Tied to the 68060's CACR they stop updating lines while EDC is off,
  and bring stale ones back when it is switched on again.
- **Core options.** Instantiated with `NO_FPU = 1` (MC68LC060) to fit beside
  the Minimig chipset; it uses the core's `dbg_vbr` and `dbg_cacr` outputs.
- **Debug capture.** `DEBUG_CAPTURE = 1` adds an In-System Sources and
  Probes instance (`A060`): a live view of the processor and both halves
  of the crossing, and the last 64 transfers; the Minimig tree's
  `tests/ap060/ap060_dbgread.tcl` reads it with `quartus_stp`.

Its test bench needs the Minimig tree (the real `cpu_wrapper`, `ddram_ctrl`,
the A2065 mailbox): it lives there, in `tests/ap060/` (`run_tests.sh`; `asm/t_bw.s` measures throughput per memory region).
