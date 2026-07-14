package motordriver

/*
Veichi SD700_ECAT_V1.2_G IMotorDriver implementation.

STATUS: bring-up stage. Only PDO mapping + basic activation have been
validated against real hardware so far (drive reaches OP with the mapping
in configs/sd700.yml). Motion has not yet been tested — StandbyOpMode,
IsTargetReached, JogControlword, and ResetMultiTurn below are reasonable
CiA-402-standard starting points but are UNVERIFIED against this drive.

Key differences from Panasonic A6 / Delta / Nidec:
 1. Live default PDO mapping has NO op_mode (0x6060) in RxPDO and NO
    digital inputs (0x60FD or any vendor register) in TxPDO. Mode of
    operation is set once via SDO during "configure" (see sd700.yml)
    rather than cyclically. Because op_mode is absent from the YAML pdo:
    entries, dev.PdoRxReady will be false until/unless 0x6060 is added
    to the mapping in a future revision — jog/position PDO control is
    therefore not yet enabled for this drive; SDO-based motion paths
    apply until that is done and validated.
 2. No vendor-specific objects at all (no 0x4F25, no 0x4D00/0x4D01,
    no 0x60FE in the current mapping). 0x4F25 does not exist anywhere
    in this drive's object dictionary.
 3. Digital inputs, POT/NOT hardware limit protection, clamp/declamp,
    and ECS are all no-ops for now (no signal source wired yet) — see
    ioStatusListener below. This means hardware POT/NOT protection is
    NOT active for this drive until 0x60FD (or equivalent) is added to
    the TxPDO mapping and wired here.
*/

import (
	ethercatDevice "EtherCAT/ethercatdevicedatatypes"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"fmt"
	"sync"
	"time"
)

// SD700 is the IMotorDriver implementation for the Veichi SD700_ECAT_V1.2_G drive.
type SD700 struct{}

func init() {
	RegisterDriver("sd700", func() IMotorDriver { return &SD700{} })
}

// hasTargetReached polls the SDO statusword bit 10 (Target Reached).
// Used only pre-activation; post-activation should use the PDO path once
// jog/position PDO control is enabled for this drive (see file header).
func (s SD700) hasTargetReached(masterDevice *MasterDevice, action int, immediate int, operation ethercatDevice.Operation) error {
	logger.Trace("SD700 waiting for target reached")
	if len(operation.Steps) < 2 {
		return fmt.Errorf("SD700.hasTargetReached requires 2 steps in operation config")
	}
	firstStep := operation.Steps[0]
	secondStep := operation.Steps[1]

	SDODownload(masterDevice.Master, masterDevice.Position, firstStep)

	for {
		result, _ := SDOUpload2(masterDevice.Master, masterDevice.Position, secondStep)
		sw := uint16(result)
		if (sw>>10)&1 == 1 {
			logger.Trace("SD700 target reached")
			return nil
		}
		time.Sleep(1 * time.Millisecond)
	}
}

// potNotEnabled is a no-op for now — SD700's live PDO mapping has no digital
// inputs wired yet. Returns false (no limit active) rather than guessing.
func (s SD700) potNotEnabled(masterDevice *MasterDevice) (bool, error) {
	return false, nil
}

// readDeclampSignal is a no-op — SD700 has no vendor clamp/declamp signals.
func (s SD700) readDeclampSignal(masterDevice *MasterDevice, declampTiming int) (bool, error) {
	return true, nil
}

// readClampSignal is a no-op — SD700 has no vendor clamp/declamp signals.
func (s SD700) readClampSignal(masterDevice *MasterDevice, clampTiming int) (bool, error) {
	return true, nil
}

// receivedECS is a no-op — SD700 has no vendor ECS signal.
func (s SD700) receivedECS(masterDevice *MasterDevice, operation ethercatDevice.Operation, stopECSChat chan bool) int {
	return 1
}

// receivedECSZero is a no-op — SD700 has no vendor ECS signal.
func (s SD700) receivedECSZero(masterDevice *MasterDevice, operation ethercatDevice.Operation, stopECSChat chan bool) int {
	return 1
}

// sendFinishSignal is a no-op for now — SD700's live PDO mapping has no
// digital outputs wired (no 0x60FE entry in sd700.yml pdo: section yet).
func (s SD700) sendFinishSignal(masterDevice *MasterDevice, operation ethercatDevice.Operation) error {
	logger.Warn("[HAL-SD700] sendFinishSignal: no digital output mapped for this drive yet — no-op")
	return nil
}

var sd700IOStopChansMu sync.Mutex
var sd700IOStopChans []chan struct{}

// pollIOStat starts one I/O status listener goroutine per device.
func (s SD700) pollIOStat(availableDevices []*MasterDevice) {
	logger.Debug("starting SD700 I/O status listener")
	sd700IOStopChansMu.Lock()
	sd700IOStopChans = make([]chan struct{}, 0, len(availableDevices))
	sd700IOStopChansMu.Unlock()

	for _, dev := range availableDevices {
		ch := make(chan struct{})
		sd700IOStopChansMu.Lock()
		sd700IOStopChans = append(sd700IOStopChans, ch)
		sd700IOStopChansMu.Unlock()
		go s.sd700IOStatusListener(dev, ch)
	}
}

// stopPollIOStat stops ALL SD700 I/O status listener goroutines.
func (s SD700) stopPollIOStat() {
	sd700IOStopChansMu.Lock()
	defer sd700IOStopChansMu.Unlock()
	for _, ch := range sd700IOStopChans {
		close(ch)
	}
	sd700IOStopChans = nil
}

// sd700IOStatusListener publishes I/O status from the PDO statusword.
// NOTE: masterDev.PDODI is not populated for this drive (no digital_in
// entry in sd700.yml pdo: section), so POT/NOT/HOME always read false here.
// This is intentionally inert rather than guessing — hardware limit
// protection is NOT active for this drive until a digital input object
// (e.g. 0x60FD) is added to the TxPDO mapping and this listener is updated
// to match, the same way A6Minas/Nidec read their respective DI registers.
func (s SD700) sd700IOStatusListener(masterDev *MasterDevice, stop <-chan struct{}) {
	for {
		select {
		case <-stop:
			logger.Debug("stopping SD700 I/O status listener for:", masterDev.Name)
			return
		default:
		}

		var ioStat statusnotifier.IOStatus
		// ALMOUT: drive fault bit (statusword bit 3) — this IS live, since
		// status_word is mapped in the TxPDO.
		ioStat.ALMOUT = (uint16(masterDev.PDOStatus.Load()&0xFFFF) & 0x0008) != 0

		driverState := getCurrentDriverStatus(masterDev.Name)
		ioStat.FIN = driverState.isSendingFinSignal
		ioStat.SOLOP = driverState.isDriverOnOff

		statusnotifier.NotifyIOStatus(ioStat)

		interval := 1000 // default 1 ms
		if masterDev.Device.IOPollingInterval > 0 {
			interval = masterDev.Device.IOPollingInterval
		}
		time.Sleep(time.Duration(interval) * time.Microsecond)
	}
}

// ============================================================
// IMotorDriver implementation
// ============================================================

// SetupPDO uses the generic YAML-driven C engine — reads configs/sd700.yml's
// pdo: section (0x1601 Rx / 0x1A01 Tx, matching the drive's live default).
func (s SD700) SetupPDO(dev *MasterDevice) error {
	return setupPDOPositionGeneric(dev)
}

// IsTargetReached returns true when a Profile Position move is complete.
// UNVERIFIED for SD700 — standard CiA-402 bit 10 assumed pending hardware
// confirmation. If SD700 requires a Set-Point Acknowledge handshake like
// the A6B (bit10=1 AND bit12=0), this will need updating.
func (s SD700) IsTargetReached(sw uint16) bool {
	const bitTargetReached = uint16(1 << 10)
	return sw&bitTargetReached != 0
}

// JogControlword returns the controlword for jog operations.
// UNVERIFIED for SD700 — clears Halt bit (8) per CiA-402 standard.
// NOTE: jog PDO control is not yet enabled for this drive (see file header) —
// this method is not on the active path until op_mode is added to the
// RxPDO mapping and validated.
func (s SD700) JogControlword(cwBase uint16) uint16 {
	return cwBase & ^uint16(0x0100)
}

// FaultResetControlword returns 0x0080 (bit 7 alone) per CiA-402 standard.
// UNVERIFIED for SD700 — if fault reset doesn't clear reliably, check
// whether this drive needs bits 0-3 set simultaneously like the Delta does.
func (s SD700) FaultResetControlword() uint16 {
	return 0x0080
}

// StandbyOpMode returns Mode 1 (Profile Position), matching the mode set
// via SDO in sd700.yml's configure sequence. UNVERIFIED — this drive's
// RxPDO does not currently carry op_mode cyclically (see file header), so
// this value is not actually written every cycle yet; it documents the
// mode the drive is left in after configure.
func (s SD700) StandbyOpMode() int8 {
	return 1
}

// ResetMultiTurn is not yet implemented for SD700 — the reset mechanism
// (if any) has not been identified in the Veichi manual yet. Returns
// ErrNotSupported rather than guessing at an SDO sequence.
func (s SD700) ResetMultiTurn(availableDevices []*MasterDevice) error {
	return ErrNotSupported
}
