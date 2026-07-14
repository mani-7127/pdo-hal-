package motordriver

//Listen for any action request on Motor to perform from the client. For e.g. reset, zero reference, etc

import (
	channels "EtherCAT/channels"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"strconv"
)

// rotation_direction is a package-level variable used to store the last jog direction.
var rotation_direction int

func initDriverActionListener() {
	logger.Debug("starting driver action listener")
	channels.DriverActionChannel = make(chan channels.DriverAction, 100)
	channels.DriveActionChannelReady()
	go listenDriverAction()
}

func stopDriverActionListener() {
	channels.NotifyMotorDriver("EXIT_DRIVE_LISTENER", "", "", 0)
}

// getDeviceForAction returns the MasterDevice that should handle a given action.
//
// Routing logic:
//   - If msg.DriveName is set (e.g. "A", "B"), find the device with that name.
//   - If msg.DriveName is empty (single-axis, or UI didn't specify),
//     fall back to devices[0] so existing behaviour is unchanged.
//
// DriveName is already a field on channels.DriverAction — when the socket
// server or GPIO handler populates it, this function routes automatically.
// No further code changes needed to enable multi-axis routing.
func getDeviceForAction(msg channels.DriverAction) *MasterDevice {
	devices := getMasterDevices()
	if len(devices) == 0 {
		return nil
	}
	if msg.DriveName != "" {
		for _, d := range devices {
			if d.Name == msg.DriveName {
				return d
			}
		}
		// Named device not found — log and fall back to devices[0].
		logger.Warn("[ACTION] device not found for DriveName:", msg.DriveName,
			"— falling back to devices[0]")
	}
	// Default: single-axis or UI without DriveName → devices[0].
	return devices[0]
}

// listenDriverAction listens on DriverActionChannel and routes each action to
// the correct MasterDevice. The device is resolved fresh on every message so
// that a system reset (which rebuilds masterDevices) never leaves a stale pointer.
func listenDriverAction() {
	for {
		msg := <-channels.DriverActionChannel

		switch msg.Action {
		case channels.RESET:
			performSysReset(true)

		case channels.MANUAL_JOG:
			device := getDeviceForAction(msg)
			if device == nil {
				logger.Error("[MANUAL_JOG] no device available")
				continue
			}
			rotation_direction = msg.Direction
			notifyDriverStatusWithWait("rotation_direction", strconv.Itoa(msg.Direction), device)
			if err := ManualJog(device, msg.Direction); err != nil {
				logger.Error("[MANUAL_JOG] failed:", err)
				statusnotifier.Alarm(err.Error())
			}

		case channels.STOP_JOG:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			// StopJog internally ramps velocity to 0, then waits 250ms for the
			// motor to physically decelerate before disabling jog mode.
			// No ResetDriver/PDOFaultReset here — the drive is NOT faulted after
			// a normal jog stop (statusword bit3=0, confirmed in every stop log).
			StopJog(device)

		case channels.ZERO_REF:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			go moveToZero(device)

		case channels.STEP_MODE_ENABLE:
			logger.Debug("step mode enabled - configureDriver bypassed")

		case channels.STEP_MODE:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			pos, _ := strconv.ParseFloat(msg.Value, 64)
			notifyDriverStatusWithWait("rotation_direction", strconv.Itoa(msg.Direction), device)
			stepMode(device, pos)

		case channels.SET_RPM:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			rpm, _ := strconv.ParseInt(msg.Value, 0, 32)
			setRpm(device, int(rpm))

		case channels.MOVE_TO_POSITION:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			degree, _ := strconv.ParseFloat(msg.Value, 64)
			// run as goroutine so this listener can continue receiving events
			// such as emergency stop while the move is in progress.
			go moveMotorToDegree(device, degree)

		case channels.START_EXECUTION:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			logger.Trace("program exec started")
			notifyDriverStatus("reset", "", device)

		case channels.POSITION_MODE:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			notifyDriverStatus("mode", msg.Value, device)

		case channels.SHORTEST_PATH_ENABLED:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			notifyDriverStatus("shortest_path_enable", msg.Value, device)

		case channels.EMERGENCY:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			stopECSCheck()
			emergency(device)
			PowerOffAll(getMasterDevices())
			statusnotifier.SocketMessage("emergency_done", "emergency completed")
			statusnotifier.Alarm("Software Emergency pressed")

		case channels.PROGRAM_EXEC_COMPLETED:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			logger.Trace("program exec completed")
			notifyDriverStatus("reset", "", device)

		case channels.FAST_POWER_OFF:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			FastPowerOff(device)

		case channels.RESET_MULTI_TURN:
			if err := resetMultiTurn(getMasterDevices()); err != nil {
				logger.Error("[RESET_MULTI_TURN] failed:", err)
			}

		case channels.SET_WORK_OFFSET:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			notifyDriverStatusWithWait("workoffset", msg.Value, device)

		case channels.SETTINGS_CHANGED:
			applyClampIfSettingsChanged()

		case channels.STOP_PROGRAM_EXECUTION:
			device := getDeviceForAction(msg)
			if device == nil {
				continue
			}
			// DESIGN: "stop after completing current command"
			//
			// The user expects the motor to finish its current move and THEN stop —
			// not to freeze mid-rotation. This matches the behaviour of the old SDO
			// version: the executor sets its internal stop flag, the running
			// moveMotorToDegree goroutine completes hasTargetReached → sends fin
			// signal → calls doneDriverAction(), then the executor checks its stop
			// flag and does not start the next line.
			//
			// What we must NOT do: cancel the active PDO position move by calling
			// EnablePosPDO(false) + SetTargetPositionPDO(current). That was the bug
			// that caused the motor to freeze mid-move (log showed stop at 284.358°
			// when target was 270°, and 249.159° when target was 0°).
			stopECSCheck()
			if IsPDOActive() && device.PdoReady {
				if device.IsJogEnabled() {
					_ = StopJog(device)
					_ = device.SetTargetVelocityPDO(0)
					logger.Info("[PDO] STOP_PROGRAM_EXECUTION: jog stopped immediately")
				}
				if device.IsPosEnabled() {
					logger.Info("[PDO] STOP_PROGRAM_EXECUTION: position move allowed to complete before stopping")
				}
			}

		case channels.EXIT_DRIVE_LISTENER:
			// 'return' exits the goroutine entirely.
			// 'break' would only exit the switch, leaving the for loop running
			// forever and leaking this goroutine on every reset.
			logger.Debug("stopping driver action listener")
			return

		default:
			logger.Error("listenDriverAction->unrecognized driver action type passed", msg.Action)
		}
	}
}

// doneDriverAction feeds back to command handlers that a command has completed,
// e.g. move to 90 degrees — once move completed, feed back that it's done so
// command handlers can execute the next line.
func doneDriverAction() {
	channels.NotifyCmdComplete()
}
