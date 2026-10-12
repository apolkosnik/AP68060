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
- **Bus.** Each 68060 transfer becomes TG68K-style 16-bit accesses (a long
  word is two, `longword` high on the first, as Gayle's 32-bit IDE port
  expects). Lines get TBI, so the processor finishes them as long-word
  transfers. Reads outside Fast RAM and Kickstart ROM get TCI: chip RAM,
  I/O and the RTG frame buffer are never cached.
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

Its test bench needs the Minimig tree (the real `cpu_wrapper`, `ddram_ctrl`,
the A2065 mailbox): it lives there, in `tests/ap060/` (`run_tests.sh`).
