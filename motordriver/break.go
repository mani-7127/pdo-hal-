package motordriver

import "EtherCAT/logger"

// breakBit is the 0x60FE bit that controls the brake solenoid digital output.
// Bit 1 of 0x60FE:02 drives OUT2 (brake solenoid wire on this machine).
const breakBit = uint32(0x00000002)

// Helper to grab the correct pointer for multi-axis targeting
func getRealDeviceForBrake(name string) *MasterDevice {
	for _, dev := range getMasterDevices() {
		if dev.Name == name {
			return dev
		}
	}
	return nil
}

func breakOn(masterDevice *MasterDevice) error {
	logger.Debug("break on")
	if IsPDOActive() {
		if masterDevice.PdoDigOutReady {
			if realDev := getRealDeviceForBrake(masterDevice.Name); realDev != nil {
				PDOSetDigitalOutput([]*MasterDevice{realDev}, breakBit, 0xFFFFFFFF)
				logger.Info("[PDO] breakOn: brake asserted via 0x60FE PDO")
				return nil
			}
		}
		logger.Warn("[PDO] breakOn: PdoDigOutReady=false, falling back to SDO (may be slow)")
	}
	operation, err := GetEtherCATOperation("break_on", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		SDODownload(masterDevice.Master, masterDevice.Position, step)
	}
	return nil
}

func breakOff(masterDevice *MasterDevice) error {
	logger.Debug("break off")
	if IsPDOActive() {
		if masterDevice.PdoDigOutReady {
			if realDev := getRealDeviceForBrake(masterDevice.Name); realDev != nil {
				PDOSetDigitalOutput([]*MasterDevice{realDev}, 0, 0xFFFFFFFF)
				logger.Info("[PDO] breakOff: brake released via 0x60FE PDO")
				return nil
			}
		}
		logger.Warn("[PDO] breakOff: PdoDigOutReady=false, falling back to SDO (may be slow)")
	}
	operation, err := GetEtherCATOperation("break_off", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		SDODownload(masterDevice.Master, masterDevice.Position, step)
	}
	return nil
}
