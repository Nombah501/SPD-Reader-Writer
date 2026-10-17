# SPD-RW
Arduino-based SPD Reader/Writer with Reversible Software Write Protection capabilities

> **Fork note (branch `kk-cudimm`).** Drop-in replacement for `spdrwcore.dll` 2.23.12.10 (same name and version, not
> strong-named, so the stock `spdrwgui.exe`/`spdrwcli.exe` 20231210 load it; all 167 GUI and 51 CLI references verified).
> Library-only fixes:
> - DDR5 module types CUDIMM, CSO-DIMM, MRDIMM, CAMM2 were missing: CUDIMM modules crashed the GUI with `IndexOutOfRangeException`.
> - `Data.GetEnumDescription` no longer throws on values without an enum member (returns `Unknown (0x…)`).
> - DDR5 "Solder down" shared code `0b1010` with DDIMM; it is `0b1011`.
> - DDR5 dies per package read bits 7:4 instead of 7:5 (DDP reported 4 dies, 8H 3DS 10 dies).
> - DDR5 `IoWidth` returned the raw code instead of the width; `TotalModuleCapacity` now in bytes like DDR3/DDR4.
> - Single-byte Arduino writes use the one-byte WritePage command (KK_SPD-V5.3 firmware rejects WriteByte).
>
> Build: `dotnet build src/SpdReaderWriterCore -c Release` (net40, x86), then replace `spdrwcore.dll` next to `spdrwgui.exe`.

<p align="center">
<img src="images/spdrwgui_1350775.png?raw=true" alt="GUI screenshot">
</p>

<p align="center">
<img src="images/spdrwgui_13636626.png?raw=true" alt="GUI screenshot">
</p>