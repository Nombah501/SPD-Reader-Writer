/*
    Arduino based EEPROM SPD reader and writer
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   For overclockers and PC hardware enthusiasts

   Repos:   https://github.com/1a2m3/SPD-Reader-Writer
   Support: https://forums.evga.com/FindPost/3053544
   Donate:  https://paypal.me/mik4rt3m

*/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using static SpdReaderWriterCore.Data;
using static SpdReaderWriterCore.DeviceId;
using static SpdReaderWriterCore.Eeprom.EepromCommand;
using static SpdReaderWriterCore.IoPort;
using static SpdReaderWriterCore.Kernel;
using static SpdReaderWriterCore.PciConfig;
using static SpdReaderWriterCore.Smbus.Platform;
using static SpdReaderWriterCore.Smbus.SmbusAccessMode;
using static SpdReaderWriterCore.VendorId;

namespace SpdReaderWriterCore {

    /// <summary>
    /// SMBus class
    /// </summary>
    public class Smbus : IDisposable, IDevice {

        /// <summary>
        /// Smbus PCI Device Info
        /// </summary>
        public PciInfo PciDeviceInfo {
            get {
                if (_pciDeviceInfo.VendorId == 0 &&
                    (_pciDeviceInfo.DeviceId == 0 || _pciDeviceInfo.DeviceId == Invalid )) {
                    _pciDeviceInfo = GetPciInfo();
                }

                return _pciDeviceInfo;
            }
        }
        private PciInfo _pciDeviceInfo;

        /// <summary>
        /// SMBus bus number
        /// </summary>
        public byte BusNumber {
            get => _busNumber;
            set {
                if (!IsConnected) {
                    return;
                }

                _busNumber = value;

                // Reset Eeprom page when bus is set
                Eeprom.ResetPageAddress(this);

                // Force SPD size and DDR5 flag update
                I2CAddress = _i2CAddress;
            }
        }
        private byte _busNumber;

        /// <summary>
        /// Current I2C address
        /// </summary>
        public byte I2CAddress {
            get => _i2CAddress;
            set {
                if (!IsConnected) {
                    return;
                }

                _i2CAddress = value;

                if (!Eeprom.ValidateEepromAddress(_i2CAddress)) {
                    return;
                }

                // Check for DDR5 presence
                IsDdr5Present = ProbeAddress((byte)(Eeprom.LidCode.Pmic0 << 3 | (Eeprom.Spd5Register.LocalHid & value))) &&
                                I2CRead(_i2CAddress, 00) == 0x51 &&
                                MatchBitmask(I2CRead(_i2CAddress, 01), 0x08);

                // Reset Eeprom page
                Eeprom.ResetPageAddress(this);
            }
        }
        private byte _i2CAddress;

        /// <summary>
        /// DDR5 presence flag
        /// </summary>
        public bool IsDdr5Present { get; private set; }

        /// <summary>
        /// Maximum SPD size on this device
        /// </summary>
        public ushort MaxSpdSize {
            get {
                if (Eeprom.ValidatePmicAddress(_i2CAddress)) {
                    return Spd.DataLength.Minimum;
                }

                if (!Eeprom.ValidateEepromAddress(_i2CAddress)) {
                    return Spd.DataLength.Unknown;
                }

                if (IsDdr5Present) {
                    return Spd.DataLength.DDR5;
                }
                // Read dram device type byte
                byte ramTypeByte = I2CRead(_i2CAddress, 2);

                // SPD header for GetRamType
                byte[] spdHeader = { 0x00, 0x00, ramTypeByte };

                // Check if dram device type byte value is in the Ram.Type enum
                return (ushort)(Enum.IsDefined(typeof(Spd.RamType), (Spd.RamType)ramTypeByte)
                    ? Spd.GetSpdSize(Spd.GetRamType(spdHeader))
                    : Spd.DataLength.Minimum); // DDR3 and older
            }
        }

        /// <summary>
        /// SPD write disable BIOS state (Intel ICH/PCH only)
        /// </summary>
        public bool SpdWriteDisabled => IsConnected &&
                                        PlatformType == Default &&
                                        PciDeviceInfo.VendorId == Intel &&
                                        GetBit(PciDevice?.Read<byte>(0x40), 4);

        /// <summary>
        /// PCI device instance
        /// </summary>
        public PciConfig PciDevice { get; private set; }

        /// <summary>
        /// IO port instance
        /// </summary>
        public IoPort IoPort { get; private set; }

        /// <summary>
        /// Initialize SMBus with default settings
        /// </summary>
        public Smbus() => Initialize(0);

        /// <summary>
        /// Initialize SMBus with specified bus number
        /// </summary>
        /// <param name="busNumber">Bus number</param>
        public Smbus(byte busNumber) => Initialize(busNumber);

        /// <summary>
        /// SMBus instance destructor
        /// </summary>
        ~Smbus() => Dispose();

        /// <summary>
        /// Established SMBus connection
        /// </summary>
        /// <returns><see langword="true"/> upon successful initialization</returns>
        public bool Connect() => Initialize(0);

        /// <summary>
        /// Deinitializes SMBus instance
        /// </summary>
        /// <returns><see langword="true"/> upon successful disposal</returns>
        public bool Disconnect() {
            Dispose();

            return !Driver.IsRunning;
        }

        /// <summary>
        /// Disposes SMBus instance
        /// </summary>
        public void Dispose() {
            Driver.Stop();
            Driver.Dispose();
        }

        /// <summary>
        /// SMBus name
        /// </summary>
        /// <returns>SMBus name in a form of platform vendor name and chipset model name</returns>
        public override string ToString() => 
            $"{GetEnumDescription(PciDeviceInfo.VendorId)} {GetEnumDescription(PciDeviceInfo.DeviceId)} {BusNumberPrefix}{BusNumber:D}{BusNumberSuffix}";

        /// <summary>
        /// Bus number string prefix
        /// </summary>
        public static string BusNumberPrefix = "#";

        /// <summary>
        /// Bus number string suffix
        /// </summary>
        public static string BusNumberSuffix = "";

        /// <summary>
        /// Describes SMBus connection state
        /// </summary>
        public bool IsConnected => _platformDetected &&
                                   _platformType != Unknown;

        /// <summary>
        /// Available SMBuses
        /// </summary>
        public byte[] SmBuses => FindBus();

        /// <summary>
        /// Available i2c addresses on selected bus
        /// </summary>
        public byte[] Addresses => Scan();

        /// <summary>
        /// Intel ICH/PCH and AMD FCH SMBus controller register offsets
        /// </summary>
        internal struct DefaultSmbusRegister {

            /// <summary>
            /// Host status
            /// </summary>
            internal const byte Status = 0x00;

            /// <summary>
            /// Host control
            /// </summary>
            /// <remarks>Used to execute <see cref="SmbusCmd"/> after setting registers <see cref="Address"/>,
            /// <see cref="HostCmd"/>, (and <see cref="Data0"/>, if <see cref="Write"/> mode is used)</remarks>
            internal const byte Control = 0x02;

            /// <summary>
            /// Host command
            /// </summary>
            /// <remarks>This registers is used to specify offset</remarks>
            internal const byte HostCmd = 0x03;

            /// <summary>
            /// Transmit slave address
            /// </summary>
            /// <remarks>Bits 7-1: i2c address;
            /// bit 0: r/w mode (1 = <see cref="Read"/>, 0 = <see cref="Write"/>)</remarks>
            /// <example>Set to 0xA1 to read from 0x50, or 0xA0 to write to 0x50</example>
            internal const byte Address = 0x04;

            /// <summary>
            /// Data 0
            /// </summary>
            /// <remarks>Input for <see cref="Write"/>,
            /// or output for <see cref="Read"/> operation</remarks>
            internal const byte Data0 = 0x05;
        }

        /// <summary>
        /// Intel CPU SMBus Device ID
        /// </summary>
        internal enum SkylakeXDeviceId : ushort {
            // LGA 2066 SKL-X & CLX-X IMC Smbus
            SkylakeXImcSmbus = 0x2085,
        }

        /// <summary>
        /// Intel Skylake-X CPU SMBus controller register offsets
        /// </summary>
        private struct SkylakeXSmbusRegister {
            internal const byte Address    = 0x9D;
            internal const byte Offset     = 0x9C;
            internal const byte Input      = 0xB6;
            internal const byte Command    = 0x9E;
            internal const byte Output     = 0xB4;
            internal const byte Status     = 0xA8;
            internal const byte DimmConfig = 0x94;
        }

        /// <summary>
        /// Describes Skylake-X CPU SMBus controller status codes at <see cref="SkylakeXSmbusRegister.Status">status register</see>
        /// </summary>
        private struct SkylakeXSmbusStatus {
            internal const byte Ready    = 0b00000000;
            internal const byte Complete = 0b00000100;
            internal const byte Busy     = 0b00000001;
            internal const byte Ack      = 0b00000000;
            internal const byte Nack     = 0b00000010;
        }

        /// <summary>
        /// Nvidia Smbus register offsets
        /// </summary>
        private struct NvidiaSmbusRegister {
            internal const byte Protocol    = 0x00;
            internal const byte Status      = 0x01;
            internal const byte Address     = 0x02;
            internal const byte Command     = 0x03; // Offset
            internal const byte Data        = 0x04; // Input (write) or output (read)
            internal const byte ByteCount   = 0x24; // Number of data bytes
            internal const byte Control     = 0x3E; // Control register
            internal const byte AbortStatus = 0x3C; // register used to check the status of the abort command
        }

        /// <summary>
        /// Nvidia Smbus protocol commands for the <see cref="NvidiaSmbusRegister.Protocol"/> register
        /// </summary>
        private struct NvidiaSmbusProtocol {
            internal const byte Quick     = 0x02;
            internal const byte Byte      = 0x04;
            internal const byte ByteData  = 0x06;
            internal const byte WordData  = 0x08;
            internal const byte BlockData = 0x0A;

            internal const byte Abort     = 0x20;

            internal const byte Pec       = 0x80;
        }

        /// <summary>
        /// Describes Nvidia Smbus status codes at <see cref="NvidiaSmbusRegister.Status">status register</see>
        /// </summary>
        private struct NvidiaSmbusStatus {
            internal const byte Done    = 0x80;
            internal const byte Invalid = 0x10;
            internal const byte Error   = 0x1F;
        }

        /// <summary>
        /// Check if the device ID is supported
        /// </summary>
        /// <param name="deviceId">Chipset DeviceID</param>
        /// <returns><see langword="true"/> if <paramref name="deviceId"/> is present in the <see cref="DeviceId"/> enum</returns>
        private bool CheckChipsetSupport(DeviceId deviceId) {
            return deviceId != Invalid &&
                   Enum.IsDefined(typeof(DeviceId), deviceId);
        }

        /// <summary>
        /// Check if the device ID belongs to Skylake-X or Cascade Lake-X platform
        /// </summary>
        /// <param name="deviceId">Chipset DeviceID</param>
        /// <returns><see langword="true"/> if <paramref name="deviceId"/> belongs to Skylake-X or Cascade Lake-X platform</returns>
        private bool IsSkylakeX(DeviceId deviceId) {

            DeviceId[] skylakexIds = {
                X299,
                C422,
            };

            return CheckChipsetSupport(deviceId) &&
                   ArrayContains(skylakexIds, deviceId);
        }

        /// <summary>
        /// Gets platform type
        /// </summary>
        private Platform PlatformType {
            get {
                if (!_platformDetected) {

                    if (IsSkylakeX(PciDeviceInfo.DeviceId)) {
                        _platformType = SkylakeX;
                    }
                    else if (CheckChipsetSupport(PciDeviceInfo.DeviceId)) {
                        switch (PciDeviceInfo.VendorId) {
                            case Nvidia:
                                _platformType = Nforce;
                                break;
                            case AMD:
                            case Intel:
                            case VIA:
                                _platformType = Default;
                                break;
                        }
                    }
                }

                _platformDetected = PciDeviceInfo.DeviceId != Invalid;

                return _platformType;
            }
        }
        private Platform _platformType;

        /// <summary>
        /// Platform detection flag
        /// </summary>
        private bool _platformDetected;

        /// <summary>
        /// Platform type
        /// </summary>
        internal enum Platform : byte {

            /// <summary>
            /// Unknown or unsupported platform type
            /// </summary>
            Unknown,

            /// <summary>
            /// Intel ICH/PCH, AMD FCH, VIA
            /// </summary>
            Default,

            /// <summary>
            /// Skylake X (incl. Refresh) and Cascade Lake X platforms
            /// </summary>
            SkylakeX,

            /// <summary>
            /// Nvidia Nforce2/3/4 and MCP
            /// </summary>
            Nforce,
        }

        /// <summary>
        /// Initializes SMBus controller class
        /// </summary>
        /// <param name="busNumber">Bus number</param>
        /// <returns><see langword="true"/> if SMBus instance is initialized successfully</returns>
        private bool Initialize(byte busNumber) {
            try {
                if (!Driver.Start()) {
                    throw new Exception($"{Driver.DriverInfo} did not start.");
                }
            }
            catch (Exception e) {
                throw new Exception($"{Driver.DriverInfo} initialization failure: ({e})");
            }

            if (!LockMutex(SmbusMutex, SmbusMutexTimeout)) {
                return false;
            }

            try {
                if (PlatformType == Unknown &&
                    PciDeviceInfo.VendorId != VIA) {
                    return false;
                }

                switch (PciDeviceInfo.VendorId) {

                    // Skylake-X
                    case Intel when PlatformType == SkylakeX:
                        // Locate CPU SMBus controller
                        PciDevice = FindDeviceById(PciDeviceInfo.VendorId, (DeviceId)SkylakeXDeviceId.SkylakeXImcSmbus);

                        if (PciDevice == null) {
                            return false;
                        }

                        break;

                    // ICH/PCH, nForce
                    case Intel:
                    case Nvidia:

                        // Find smbus controller
                        PciDevice = FindDeviceByClass(BaseClassType.Serial, SubClassType.Smbus);

                        if (PciDevice == null) {
                            return false;
                        }

                        // Enable SMBus IO Space if disabled
                        ushort command = PciDevice.Read<ushort>(Register.Command);
                        if (!GetBit(command, 1) && PciDevice.VendorId == Intel) {
                            PciDevice.Write(Register.Command, SetBit(command, 0, true));
                        }

                        // Read IO port address and info
                        ushort ioPortAddress = PciDevice.Read<ushort>(Register.BaseAddress[4]);

                        // Check if SMbus is port mapped
                        if (GetBit(ioPortAddress, 0)) {
                            // Initialize new SMBus IO port instance
                            IoPort = new IoPort(SetBit(ioPortAddress, 0, false));
                        }

                        break;

                    // AM4, AM5, AM1, FM1, FM2(+)
                    case AMD:
                    case Hygon:

                        // Find default SMBus controller(s)
                        PciDevice = FindDeviceByClass(BaseClassType.Serial, SubClassType.Smbus);

                        if (PciDevice == null) {
                            return false;
                        }

                        switch (PciDevice.DeviceId) {
                            case ZEN when PciDeviceInfo.VendorId == AMD && PciDevice.RevisionId >= 0x49:
                            case FCH when PciDeviceInfo.VendorId == AMD && PciDevice.RevisionId >= 0x41:
                            case FCH when PciDeviceInfo.VendorId == Hygon:

                                // PMIO registers accessible via IO ports
                                ushort SB800_PIIX4_SMB_IDX = 0x0CD6;
                                ushort SB800_PIIX4_SMB_DAT = 0x0CD7;

                                byte smba_en_hi; // Address
                                byte smba_en_lo; // Status

                                byte smb_en = 0x00;

                                if (!LockIoPortMutex()) {
                                    return false;
                                }

                                WriteIoPortEx(SB800_PIIX4_SMB_IDX, (byte)(smb_en + 1));
                                smba_en_hi = ReadIoPort<byte>(SB800_PIIX4_SMB_DAT);

                                WriteIoPortEx(SB800_PIIX4_SMB_IDX, smb_en);
                                smba_en_lo = ReadIoPort<byte>(SB800_PIIX4_SMB_DAT);

                                UnlockIoPortMutex();

                                if (smba_en_hi == byte.MaxValue) {
                                    // PMIO is disabled, get SMBus port from memory
                                    uint SB800_PIIX4_FCH_PM_ADDR = 0xFED80300;
                                    uint smbusBase = ReadMemory<uint>(SB800_PIIX4_FCH_PM_ADDR);

                                    IoPort = new IoPort((ushort)(((smbusBase >> BitsInByte) & 0xFF) << BitsInByte));
                                }
                                else if (smba_en_lo != byte.MaxValue && 
                                         GetBit(smba_en_lo, 4)) {
                                    // Primary bus
                                    ushort piix4_smba = (ushort)(smba_en_hi << BitsInByte); // 0x0B00

                                    if (piix4_smba == 0x00) {
                                        return false;
                                    }

                                    // Initialize new SMBus IO port instance
                                    IoPort = new IoPort(piix4_smba);
                                }

                                break;

                            default:
                                return false;
                        }

                        break;

                    case VIA:

                        // Find PCI to ISA bridge
                        PciDevice = FindDeviceByClass(BaseClassType.Bridge, SubClassType.Isa);

                        if (PciDevice == null) {
                            return false;
                        }

                        byte SMBBA1 = 0x90;
                        byte SMBBA3 = 0xD0;

                        // Assign smbus base address offset
                        byte viaSmbBa;

                        switch (PciDevice.DeviceId) {
                            case VT8233:
                            case VT8233A:
                            case VT8235:
                            case VT8237A:
                            case VT8237R:
                            case VT8237S:
                            case VT8251:
                            case CX700:
                            case VX8x0:
                            case VX8x5:
                            case VX900:
                                viaSmbBa = SMBBA3;
                                break;

                            case VT8231:
                            case VT82C596x:
                            case VT82C686x:
                                // Switch to ACPI Power Management system which includes an SMBus interface controller
                                PciDevice.Function = 4;
                                viaSmbBa = SMBBA1;
                                break;

                            default:
                                // Unsupported chipset
                                return false;
                        }

                        // Read SMBus I/O Base from base address offset
                        IoPort = new IoPort((ushort)(PciDevice.Read<ushort>(viaSmbBa) & 0xFFF0));

                        break;

                    default:
                        return false;
                }

                BusNumber = busNumber;
            }
            finally {
                UnlockMutex(SmbusMutex);
            }

            return IsConnected;
        }

        /// <summary>
        /// Get platform information
        /// </summary>
        /// <returns>Platform and chipset Device/Vendor ID</returns>
        private PciInfo GetPciInfo() {

            PciConfig platform = new PciConfig();
            PciInfo pciInfo = new PciInfo {
                DeviceId = Invalid
            };

            if (!LockPciMutex()) {
                return pciInfo;
            }

            try {
                pciInfo.VendorId = platform.VendorId;

                switch (pciInfo.VendorId) {
                    case Intel:
                    case VIA:
                        // Find ISA bridge to get chipset ID
                        foreach (PciConfig isa in FindDevicesByClass(BaseClassType.Bridge, SubClassType.Isa, 0, 1)) {
                            pciInfo.DeviceId = isa.DeviceId;
                            break;
                        }

                        break;

                    case AMD:
                    case Hygon:
                    case Nvidia:
                        // Find Smbus controller
                        foreach (PciConfig smbus in FindDevicesByClass(BaseClassType.Serial, SubClassType.Smbus, 0, 1)) {
                            pciInfo.DeviceId = smbus.DeviceId;
                            break;
                        }

                        break;
                }

            }
            finally {
                UnlockPciMutex();
            }

            return pciInfo;
        }

        /// <summary>
        /// Locates up to 2 available SMBuses on the device
        /// </summary>
        /// <returns>An array of bytes containing SMBus numbers</returns>
        public byte[] FindBus() => FindBus(2);

        /// <summary>
        /// Locates available SMBuses on the device
        /// </summary>
        /// <param name="total">Maximum number of buses to find</param>
        /// <returns>An array of bytes containing SMBus numbers</returns>
        public byte[] FindBus(byte total) {

            if (!IsConnected) {
                return new byte[0];
            }

            byte originalBus = BusNumber;

            try {
                Queue<byte> result = new Queue<byte>();

                for (byte i = 0; i < total; i++) {
                    BusNumber = i;
                    if (Scan(false).Length > 0) {
                        result.Enqueue(i); // The bus is valid
                    }
                }

                return result.Count > 1 ? UniqueArray(result.ToArray()) : result.ToArray();
            }
            catch {
                return new byte[0];
            }
            finally {
                BusNumber = originalBus;
            }
        }

        /// <summary>
        /// Validates bus address by reading first byte from it
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <returns><see langword="true"/> if <paramref name="busAddress"/> exists on the bus</returns>
        public bool ProbeAddress(byte busAddress) =>
            I2CRead(busAddress);

        /// <summary>
        /// Gets DDR5 offline mode state
        /// </summary>
        /// <returns><see langword="true"/> if DDR5 RAM is in offline mode</returns>
        public bool GetOfflineMode() =>
            I2CAddress == 80 && GetOfflineMode(I2CAddress);

        /// <summary>
        /// Gets DDR5 offline mode state
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <returns><see langword="true"/> if DDR5 at <paramref name="busAddress"/> is in offline mode</returns>
        public bool GetOfflineMode(byte busAddress) =>
            IsDdr5Present && GetBit(I2CRead(busAddress, Eeprom.Spd5Register.MR48), 2);

        /// <summary>
        /// Scan SMBus for available slave devices
        /// </summary>
        /// <returns>An array of available bus addresses</returns>
        public byte[] Scan() =>
            Scan(completeScan: true);

        /// <summary>
        /// Scan SMBus for available slave devices
        /// </summary>
        /// <param name="completeScan">Set to <see langword="true"/> to to scan the entire range,
        /// or <see langword="false"/> stop scanning once at least one slave address is found</param>
        /// <returns>An array of found bus addresses on <see cref="BusNumber"/></returns>
        private byte[] Scan(bool completeScan) {

            Queue<byte> result = new Queue<byte>();

            Driver.LockHandle();

            for (byte i = 0x50; i <= 0x57; i++) {
                if (!ProbeAddress(i)) {
                    continue;
                }

                result.Enqueue(i);

                if (!completeScan) {
                    break;
                }
            }

            Driver.UnlockHandle();

            return result.ToArray();
        }

        /// <summary>
        /// Attempts to read from specified slave address
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <returns><see langword="true"/> if SMBus responds with
        /// <see cref="SmbStatus.Success"/> or <see cref="SmbStatus.Ready"/></returns>
        public bool I2CRead(byte busAddress) {

            SmbusData busData = new SmbusData {
                BusNumber   = BusNumber,
                Address     = busAddress,
                Offset      = DNC,
                AccessMode  = Read,
                DataCommand = SmbusDataCommand.Byte
            };

            return ExecuteCommand(ref busData);
        }

        /// <summary>
        /// Reads a single byte from specified offset of specified slave address (disregarding page address)
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <param name="offset">Byte position</param>
        /// <returns>Byte value read from the device</returns>
        public byte I2CRead(byte busAddress, ushort offset) {

            SmbusData busData = new SmbusData {
                BusNumber   = BusNumber,
                Address     = busAddress,
                Offset      = offset,
                AccessMode  = Read,
                DataCommand = SmbusDataCommand.ByteData
            };

            ExecuteCommand(ref busData);
            return busData.Data;
        }

        /// <summary>
        /// Writes DNC data to the device
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <returns><see langword="true"/> if SMBus responds with
        /// <see cref="SmbStatus.Success"/> or <see cref="SmbStatus.Ready"/></returns>
        public bool I2CWrite(byte busAddress) {

            SmbusData busData = new SmbusData {
                BusNumber   = BusNumber,
                Address     = busAddress,
                Offset      = DNC,
                Data        = DNC,
                AccessMode  = Write,
                DataCommand = SmbusDataCommand.Byte
            };

            return ExecuteCommand(ref busData);
        }

        /// <summary>
        /// Write specified byte value to specified offset
        /// </summary>
        /// <param name="busAddress">I2C bus address</param>
        /// <param name="offset">Byte position</param>
        /// <param name="value">Byte value</param>
        /// <returns><see langwod="true"/> if SMBus responds with
        /// <see cref="SmbStatus.Success"/> or <see cref="SmbStatus.Ready"/></returns>
        public bool I2CWrite(byte busAddress, ushort offset, byte value) {

            SmbusData busData = new SmbusData {
                BusNumber   = BusNumber,
                Address     = busAddress,
                Offset      = offset,
                Data        = value,
                AccessMode  = Write,
                DataCommand = SmbusDataCommand.ByteData
            };

            return ExecuteCommand(ref busData);
        }

        /// <summary>
        /// Programs SMBus controller with input data 
        /// </summary>
        /// <param name="smbusData">SMBus input data</param>
        /// <returns><see langword="true"/> if SMBus doesn't return
        /// <see cref="SmbStatus.Error"/> or <see cref="SmbStatus.Timeout"/></returns>
        private bool ExecuteCommand(ref SmbusData smbusData) {

            if (PlatformType == Unknown) {
                smbusData.Status = SmbStatus.Error;
                return false;
            }

            if (!LockMutex(SmbusMutex, SmbusMutexTimeout)) {
                smbusData.Status = SmbStatus.Timeout;
                return false;
            }

            try {
                Driver.LockHandle();

                switch (PlatformType) {
                    case SkylakeX:
                        // Set input for writing
                        if (smbusData.AccessMode == Write) {
                            PciDevice.WriteEx(
                                offset : (byte)(SkylakeXSmbusRegister.Input + smbusData.BusNumber * 4),
                                value  : smbusData.Data);
                        }

                        // Set slave address, offset, and Execute command
                        PciDevice.WriteEx(
                            offset : (byte)(SkylakeXSmbusRegister.Offset + smbusData.BusNumber * 4),
                            value  : (uint)(
                                // Enable byte read
                                SetBit(int.MinValue, 29, true) /*0x20 << 24*/ |
                                // Execute command
                                (SmbusCmd.CmdByteData | SmbusCmd.Start) << 16 |
                                // Address and R/W mode
                                (smbusData.Address | (byte)(~(byte)smbusData.AccessMode << 7)) << BitsInByte |
                                // Offset
                                (byte)smbusData.Offset
                            )
                        );

                        // Wait after writing
                        if (smbusData.AccessMode == Write) {
                            Thread.Sleep(Eeprom.ValidateEepromAddress(smbusData.Address)
                                ? ExecutionDelay.WriteDelay
                                : ExecutionDelay.ReadDelay);
                        }

                        // Wait for completion
                        if (!WaitForStatus(
                                statuses : new[] { SmbStatus.Ready, SmbStatus.Success, SmbStatus.Error },
                                timeout  : StatusWaitTimeout)) {
                            smbusData.Status = SmbStatus.Timeout;
                            return false;
                        }

                        // Set status
                        smbusData.Status = GetBusStatus();

                        // Read and assign output if read mode is specified
                        if (smbusData.Status == SmbStatus.Error) {
                            return false;
                        }

                        if (smbusData.AccessMode == Read) {
                            smbusData.Data = PciDevice.Read<byte>(
                                offset : (byte)(SkylakeXSmbusRegister.Output + smbusData.BusNumber * 4));
                        }

                        break;

                    case Nforce:
                        if (smbusData.BusNumber > 0) {
                            return false;
                        }

                        // Set Smbus address
                        IoPort.WriteEx(
                            offset : NvidiaSmbusRegister.Address,
                            value  : (byte)(smbusData.Address << 1));

                        // Set offset
                        IoPort.WriteEx(
                            offset : NvidiaSmbusRegister.Command,
                            value  : (byte)smbusData.Offset);

                        // Set data byte register if Write mode is set
                        if (smbusData.AccessMode == Write) {
                            IoPort.WriteEx(
                                offset : NvidiaSmbusRegister.Data,
                                value  : smbusData.Data);
                        }

                        // Protocol command
                        byte protocolCmd = 0x00;

                        // Set protocol access mode
                        protocolCmd |= (byte)smbusData.AccessMode;

                        // Command type
                        switch (smbusData.DataCommand) {
                            case SmbusDataCommand.Quick:
                                protocolCmd |= NvidiaSmbusProtocol.Quick;
                                break;
                            case SmbusDataCommand.Byte:
                                protocolCmd |= NvidiaSmbusProtocol.Byte;
                                break;
                            case SmbusDataCommand.ByteData:
                            default:
                                protocolCmd |= NvidiaSmbusProtocol.ByteData;
                                break;
                            case SmbusDataCommand.WordData:
                                protocolCmd |= NvidiaSmbusProtocol.WordData;
                                break;
                        }

                        // Execute command
                        IoPort.WriteEx(
                            offset : NvidiaSmbusRegister.Protocol,
                            value  : protocolCmd);

                        // Wait after writing
                        if (smbusData.AccessMode == Write) {
                            Thread.Sleep(Eeprom.ValidateEepromAddress(smbusData.Address)
                                ? ExecutionDelay.WriteDelay
                                : ExecutionDelay.ReadDelay);
                        }

                        // Wait for completion
                        if (!WaitForStatus(
                                statuses : new[] { SmbStatus.Success, SmbStatus.Error },
                                timeout  : StatusWaitTimeout)) {
                            smbusData.Status = SmbStatus.Timeout;
                            return false;
                        }

                        // Set status
                        smbusData.Status = GetBusStatus();

                        if (smbusData.Status == SmbStatus.Error) {
                            return false;
                        }

                        // Read and assign output if read mode is specified
                        if (smbusData.AccessMode == Read) {
                            smbusData.Data = IoPort.Read<byte>(NvidiaSmbusRegister.Data);
                        }

                        break;

                    case Default:
                        // These platforms don't support multiple SMBuses
                        if (smbusData.BusNumber > 0 && 
                            (PciDeviceInfo.VendorId == Intel ||
                             PciDeviceInfo.VendorId == VIA)) {
                            return false;
                        }

                        byte portOffset = 0;

                        // Alternative AMD SMBus
                        if (PciDeviceInfo.VendorId == AMD) {
                            portOffset = (byte)(BusNumber * 20);
                        }

                        // Stop current transaction
                        if (!IoPort.WriteEx(
                                offset : (byte)(DefaultSmbusRegister.Control + portOffset),
                                value  : SmbusCmd.Stop)) {
                            smbusData.Status = SmbStatus.Error;
                            return false;
                        }

                        // Clear status bitmask to reset status
                        byte clearStatusMask = SmbusStatus.HostBusy |
                                               SmbusStatus.Interrupt |
                                               SmbusStatus.DeviceErr |
                                               SmbusStatus.BusCollision |
                                               SmbusStatus.Failed;
                        // Reset status
                        if (!IoPort.WriteEx(
                                offset : (byte)(DefaultSmbusRegister.Status + portOffset),
                                value  : clearStatusMask)) {
                            smbusData.Status = SmbStatus.Error;
                            return false;
                        }

                        // Wait for ready status
                        if (!WaitForStatus(
                                status  : SmbStatus.Ready,
                                timeout : StatusWaitTimeout)) {
                            smbusData.Status = SmbStatus.Timeout;
                            return false;
                        }

                        // Set slave address
                        if (!IoPort.WriteEx(
                                offset : (byte)(DefaultSmbusRegister.Address + portOffset),
                                value  : (byte)(smbusData.Address << 1 | (byte)smbusData.AccessMode))) {
                            smbusData.Status = SmbStatus.Error;
                            return false;
                        }

                        // Set input data for writing
                        if (smbusData.AccessMode == Write) {
                            if (!IoPort.WriteEx(
                                    offset : (byte)(DefaultSmbusRegister.Data0 + portOffset),
                                    value  : smbusData.Data)) {
                                smbusData.Status = SmbStatus.Error;
                                return false;
                            }
                        }

                        // Set offset
                        if (!IoPort.WriteEx(
                                offset : (byte)(DefaultSmbusRegister.HostCmd + portOffset),
                                value  : (byte)smbusData.Offset)) {
                            smbusData.Status = SmbStatus.Error;
                            return false;
                        }

                        // Command type
                        byte smbusDataCmd;
                        switch (smbusData.DataCommand) {
                            case SmbusDataCommand.Quick:
                                smbusDataCmd = SmbusCmd.CmdQuick;
                                break;
                            case SmbusDataCommand.Byte:
                                smbusDataCmd = SmbusCmd.CmdByte;
                                break;
                            case SmbusDataCommand.ByteData:
                            default:
                                smbusDataCmd = SmbusCmd.CmdByteData;
                                break;
                            case SmbusDataCommand.WordData:
                                smbusDataCmd = SmbusCmd.CmdWordData;
                                break;
                        }

                        // Execute
                        if (!IoPort.WriteEx(
                                offset : (byte)(DefaultSmbusRegister.Control + portOffset),
                                value  : (byte)(smbusDataCmd | SmbusCmd.Interrupt | SmbusCmd.Start))) {
                            smbusData.Status = SmbStatus.Error;
                            return false;
                        }

                        // Wait after writing
                        if (smbusData.AccessMode == Write) {
                            Thread.Sleep(Eeprom.ValidateEepromAddress(smbusData.Address)
                                ? ExecutionDelay.WriteDelay
                                : ExecutionDelay.ReadDelay);
                        }

                        // Wait for completion
                        if (!WaitForStatus(
                                statuses : new[] { SmbStatus.Ready, SmbStatus.Success, SmbStatus.Error },
                                timeout  : StatusWaitTimeout)) {

                            // Abort current execution
                            smbusData.Status = !IoPort.WriteEx((byte)(DefaultSmbusRegister.Control + portOffset), SmbusCmd.Stop) 
                                ? SmbStatus.Error 
                                : SmbStatus.Timeout;

                            return false;
                        }

                        // Check status
                        smbusData.Status = GetBusStatus();

                        // Check status
                        if (smbusData.Status == SmbStatus.Error) {
                            return false;
                        }

                        // Read and assign output if read mode is specified
                        if (smbusData.AccessMode == Read) {
                            smbusData.Data = IoPort.Read<byte>(offset: DefaultSmbusRegister.Data0);
                        }

                        break;

                    case Unknown:
                    default:
                        return false;
                }
            }

            catch (Exception e) {
                smbusData.Status = SmbStatus.Error;
                return false;
            }

            finally {
                Driver.UnlockHandle();
                SmbusMutex.ReleaseMutex();
            }

            return true;
        }

        /// <summary>
        /// SMBus Host Control Register data
        /// </summary>
        internal struct SmbusCmd {

            /// <summary>
            /// Initiates the Smbus command 
            /// </summary>
            internal const byte Start       = 1 << 6;

            /// <summary>
            /// Stops the current host transaction in process
            /// </summary>
            internal const byte Stop        = 1 << 1;

            /// <summary>
            /// Enables the generation of an interrupt upon the completion of the command
            /// </summary>
            internal const byte Interrupt   = 1 << 0;

            /// <summary>
            /// <b>Quick:</b>
            /// The slave address and read/write value (bit 0) are stored in the transmit slave address register.
            /// </summary>
            internal const byte CmdQuick    = 0b000 << 2;

            /// <summary>
            /// <b>Byte:</b>
            /// This command uses the transmit slave address and command registers.
            /// Bit 0 of the slave address register determines if this is a read or write command.
            /// </summary>
            internal const byte CmdByte     = 0b001 << 2;

            /// <summary>
            /// <b>Byte Data:</b>
            /// This command uses the transmit slave address, command, and <see cref="DefaultSmbusRegister.Data0"/> registers.
            /// Bit 0 of the slave address register determines if this is a read or write command.
            /// If it is a read, the <see cref="DefaultSmbusRegister.Data0"/> register will contain the read data.
            /// </summary>
            internal const byte CmdByteData = 0b010 << 2;

            /// <summary>
            /// <b>Word Data:</b>
            /// This command uses the transmit slave address, command, <see cref="DefaultSmbusRegister.Data0"/> and DATA1 registers.
            /// Bit 0 of the slave address register determines if this is a read or write command.
            /// If it is a read, after the command completes, the <see cref="DefaultSmbusRegister.Data0"/> and DATA1 registers will contain the read data.
            /// </summary>
            internal const byte CmdWordData = 0b011 << 2;
        }

        /// <summary>
        /// SMBus input data for <see cref="ExecuteCommand"/>
        /// </summary>
        internal struct SmbusData {
            internal byte BusNumber;
            internal byte Address;
            internal ushort Offset;
            internal SmbusAccessMode AccessMode;
            internal SmbusDataCommand DataCommand;
            internal byte Data;
            internal SmbStatus Status;
        }

        /// <summary>
        /// SMBus Read/Write access
        /// </summary>
        internal enum SmbusAccessMode : byte {
            Write,
            Read
        }

        /// <summary>
        /// Smbus command type
        /// </summary>
        internal enum SmbusDataCommand : byte {
            Quick,
            Byte,
            ByteData,
            WordData,
        }

        /// <summary>
        /// Command execution delays
        /// </summary>
        private struct ExecutionDelay {
            internal static readonly byte WriteDelay = 25;
            internal static readonly byte ReadDelay  = 0;
        }

        /// <summary>
        /// Status wait timeout in milliseconds
        /// </summary>
        public static int StatusWaitTimeout { get; set; } = 100;

        /// <summary>
        /// Gets SMBus status
        /// </summary>
        /// <returns>SMBus status</returns>
        public SmbStatus GetBusStatus() {

            if (!LockMutex(SmbusMutex, SmbusMutexTimeout)) {
                return SmbStatus.Timeout;
            }

            try {
                byte status;

                switch (PlatformType) {
                    case SkylakeX:
                        status = PciDevice.Read<byte>((byte)(SkylakeXSmbusRegister.Status + BusNumber * 4));

                        if (MatchBitmask(status, SkylakeXSmbusStatus.Busy)) {
                            return SmbStatus.Busy;
                        }
                        else if (MatchBitmask(status, SkylakeXSmbusStatus.Nack)) {
                            return SmbStatus.Error;
                        }
                        else if (MatchBitmask(status, SkylakeXSmbusStatus.Complete)) {
                            return SmbStatus.Success;
                        }
                        else {
                            return SmbStatus.Ready;
                        }

                    case Nforce:
                        if (IoPort.Read<byte>(NvidiaSmbusRegister.Protocol) != 0) {
                            return SmbStatus.Busy;
                        }

                        status = IoPort.Read<byte>(NvidiaSmbusRegister.Status);

                        return status == NvidiaSmbusStatus.Done
                            ? SmbStatus.Success
                            : SmbStatus.Error;

                    case Default:
                        if (!IoPort.ReadEx(DefaultSmbusRegister.Status, out status)) {
                            return SmbStatus.Unknown;
                        }

                        // Check busy flag
                        if (MatchBitmask(status, SmbusStatus.HostBusy)) {
                            return SmbStatus.Busy;
                        }
                        // Check for errors
                        else if (MatchBitmask(status, SmbusStatus.Failed) ||
                                 MatchBitmask(status, SmbusStatus.BusCollision) ||
                                 MatchBitmask(status, SmbusStatus.DeviceErr)) {
                            return SmbStatus.Error;
                        }
                        // Check Interrupt flag (completion)
                        else if (MatchBitmask(status, SmbusStatus.Interrupt)) {
                            return SmbStatus.Success;
                        }
                        else {
                            return SmbStatus.Ready;
                        }
                }

                return SmbStatus.Unknown;
            }
            finally {
                UnlockMutex(SmbusMutex);
            }
        }

        /// <summary>
        /// SMBus status timeout handler
        /// </summary>
        /// <param name="status">Desired SMBus status</param>
        /// <param name="timeout">Timeout in milliseconds</param>
        /// <returns><see langword="true"/> if SMBus status matches <paramref name="status"/> before timeout occurs</returns>
        private bool WaitForStatus(SmbStatus status, int timeout) =>
            WaitForStatus(new[] { status }, timeout);

        /// <summary>
        /// SMBus status timeout handler
        /// </summary>
        /// <param name="statuses">An array of SMBus statuses to wait for</param>
        /// <param name="timeout">Timeout in milliseconds</param>
        /// <returns><see langword="true"/> if SMBus status matches one of <paramref name="statuses"/> before timeout occurs</returns>
        private bool WaitForStatus(SmbStatus[] statuses, int timeout) {

            Stopwatch sw = Stopwatch.StartNew();

            do {
                SmbStatus currentStatus = GetBusStatus();

                foreach (SmbStatus status in statuses) {
                    if (currentStatus == status) {
                        return true;
                    }
                }
            } while (sw.ElapsedMilliseconds < timeout);

            return false;
        }

        /// <summary>
        /// SMBus status
        /// </summary>
        public enum SmbStatus : byte {
            Unknown, // The operation did not start
            Ready,
            Busy,
            Error,
            Success,
            Timeout,
            Aborted,
        }

        /// <summary>
        /// Host status (<see cref="DefaultSmbusRegister.Status"/>) register bits description
        /// </summary>
        internal struct SmbusStatus {

            /// <summary>
            /// Indicates the SMBus controller is in the process of completing a command
            /// </summary>
            internal const byte HostBusy     = 1 << 0;

            /// <summary>
            /// Indicates the successful completion of the last host command
            /// </summary>
            internal const byte Interrupt    = 1 << 1;

            /// <summary>
            /// Indicates an error: illegal command, unclaimed cycle, or timeout
            /// </summary>
            internal const byte DeviceErr    = 1 << 2;

            /// <summary>
            /// Indicates an SMBus transaction collision
            /// </summary>
            internal const byte BusCollision = 1 << 3;

            /// <summary>
            /// Indicates a failed bus transaction
            /// </summary>
            internal const byte Failed       = 1 << 4;
        }

        /// <summary>
        /// Global SMBus mutex
        /// </summary>
        internal static Mutex SmbusMutex = CreateMutex(SmbusMutexName);

        /// <summary>
        /// Global SMBus mutex name
        /// </summary>
        internal static string SmbusMutexName => "Global\\Access_SMBUS.HTP.Method";

        /// <summary>
        /// Mutex wait timeout in milliseconds
        /// </summary>
        public static int SmbusMutexTimeout { get; set; } = 1000;
    }
}