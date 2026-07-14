package motordriver

import (
	logger "EtherCAT/logger"
	settings "EtherCAT/settings"
	"errors"
	"fmt"
)

// hasDeclamped verifies whether declamping happened.
//
// FIX (Bug 2a): Changed signature from (MasterDevice) to (*MasterDevice).
//
// The old value-copy signature had two problems:
//  1. masterDevice.Driver is an interface pointer. When the struct is copied,
//     the interface value is copied correctly — but any method that inspects
//     the pointer identity of the device (e.g. to match against masterDevices[])
//     would see a different address. More importantly it signals intent wrongly:
//     this function reads live PDO state from the device which must be the
//     real struct, not a snapshot.
//  2. The GetMotorDriver() fallback returned the global driver — always the
//     last drive type configured via SetMotorDriver(). On a mixed-axis setup
//     (A6 on Axis A, Delta on Axis B) this routed Axis A's declamp signal
//     read through Delta's bit-mapping, giving wrong HIGH/LOW results.
//
// FIX (Bug 2b): Replace GetMotorDriver() fallback with a hard error.
//
//	The fallback silently masked the real problem (Driver not set at InitMaster).
//	A clear error forces the bug to surface immediately instead of producing
//	wrong clamp/declamp readings that are impossible to diagnose in the field.
func hasDeclamped(masterDevice *MasterDevice, envSettings settings.DriverSettings) (bool, error) {
	if masterDevice == nil {
		return false, errors.New("hasDeclamped: nil device")
	}
	driver := masterDevice.Driver
	if driver == nil {
		// Hard error — Driver must always be set by InitMaster before any
		// motion function is called. If this fires, the bug is in InitMaster,
		// not here. A silent fallback to GetMotorDriver() would use the wrong
		// driver HAL and produce incorrect clamp signal readings.
		return false, fmt.Errorf("hasDeclamped: Driver not initialised for device %s — check InitMaster", masterDevice.Name)
	}
	if envSettings.ClampDeclamp == 1 {
		breakOff(masterDevice)
		logger.Debug("Clamp declamp enabled, checking the status")
		isDeclamped, clampErr := driver.readDeclampSignal(masterDevice, envSettings.ClampDeclampTiming)
		return isDeclamped, clampErr
	}
	return true, nil
}

// hasClamped verifies whether clamping happened.
// Returns true if clamped successfully, false with error if not clamped.
//
// FIX (Bug 2a): Changed signature from (MasterDevice) to (*MasterDevice).
// FIX (Bug 2b): Replace GetMotorDriver() fallback with a hard error.
// See hasDeclamped above for the full explanation.
func hasClamped(masterDevice *MasterDevice, envSettings settings.DriverSettings) (bool, error) {
	if masterDevice == nil {
		return false, errors.New("hasClamped: nil device")
	}
	driver := masterDevice.Driver
	if driver == nil {
		return false, fmt.Errorf("hasClamped: Driver not initialised for device %s — check InitMaster", masterDevice.Name)
	}
	if envSettings.ClampDeclamp == 1 {
		breakOn(masterDevice)

		// Always call FastPowerOff — it is now PDO-safe.
		// When PDO is active, FastPowerOff sets pdoShutdownActive=true so the
		// cyclic task sends CW=0x0006 (Shutdown), walking the drive from
		// Operation Enabled → Ready To Switch On without any SDO/PDO race.
		// The physical clamp solenoid needs the drive de-energised before the
		// CL input (bit 6) goes high — this was the root cause of clamp failures.
		//
		// FIX (Bug 2a): Pass *masterDevice not a copy. FastPowerOff internally
		// looks up the device by name from masterDevices[] to call
		// pdoShutdownActive.Store(true). Passing a copy worked because the name
		// string matches — but the pointer receiver makes the intent explicit and
		// prevents future refactors from accidentally breaking the lookup.
		if err := FastPowerOff(masterDevice); err != nil {
			return false, err
		}

		logger.Debug("Clamp declamp enabled, checking the status")
		isDeclamped, clampErr := driver.readClampSignal(masterDevice, envSettings.ClampDeclampTiming)
		if clampErr != nil {
			return false, clampErr
		}
		if !isDeclamped {
			return false, errors.New("Clamping error")
		}
		// clamping is enabled and clamp activated, return true
		return true, nil
	}
	// clamping not enabled, return false.
	return false, nil
}

// applyClampIfSettingsChanged applies clamping when settings change and
// clamp/declamp is enabled, provided the motor is not currently running.
func applyClampIfSettingsChanged() {
	for _, dev := range masterDevices {
		devSetting := settings.GetDriverSettings(dev.Name)
		stat := getCurrentDriverStatus(dev.Name)
		if stat.isMotorRunning {
			return
		}

		if devSetting.ClampDeclamp == 1 {
			logger.Debug("CL/DL enabled power off driver", dev.Name)
			FastPowerOff(dev)
		} else {
			logger.Debug("CL/DL disabled power on driver", dev.Name)
			FastPowerOn(dev)
		}
	}
}
