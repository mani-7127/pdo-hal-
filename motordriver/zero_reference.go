package motordriver

import (
	channels "EtherCAT/channels"
	logger "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	"EtherCAT/settings"
	"fmt"
	"math"
	//"time"
)

// zeroRefFeed is the fixed feed rate used exclusively for zero-reference moves.
// This is intentionally decoupled from JogFeed so the customer can freely
// change jog speed without affecting the homing move speed.
// Unit: same as JogFeed (multiplied by RPMConst inside setRpm to get counts/sec).
const zeroRefFeed = 10

// moveToZero moves the motor to the absolute zero reference position using
// the shortest path.
//
// FIX (Bug 15): The old version called configureDriver(device) immediately
// before freeRotate.  configureDriver sends SDO initialisation sequences
// (operating mode, gains, limits …) to the drive.  Calling it during live
// PDO-cyclic operation is unsafe: the IgH EtherCAT master does not serialise
// SDO requests against the cyclic send/receive, so the two can collide and
// produce frame errors or drive faults.
//
// configureDriver belongs in the one-time startup sequence (InitMaster) where
// the master has not yet been activated.  It must NOT be called from any
// motion function.  It has been removed here; the drive is already correctly
// configured at boot time.
func moveToZero(device *MasterDevice) error {
	// FIX: motionMu was previously only acquired by moveMotorToDegree, even
	// though its own comment says it exists to prevent ANY two motion
	// goroutines from touching shared device state concurrently.
	// moveToZero calls the exact same underlying primitives (SetTargetPositionPDO,
	// hasTargetReached, ppSetpointPending) on the same *MasterDevice — without
	// this lock, a zero-reference move and a position move could run fully
	// concurrently and race on that shared state. This matches the observed
	// bug pattern exactly: every failing test sequence had a zero-reference
	// move immediately preceding the position move that then failed with a
	// confusing "false target reached" error. Fully generic — applies to
	// every drive, not just SD700.
	motionMu.Lock()
	defer motionMu.Unlock()

	logger.Debug("move to zero started")

	driverStatus := getCurrentDriverStatus(device.Device.Name)
	envSettings := settings.GetDriverSettings(device.Name)

	const targetPosition = 0.0
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.0), device)
	notifier.NotifyDestinationPosition(device.Name, float32(targetPosition))
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", targetPosition), device)

	// --- RS232 PROCEDURE: choose direction using HomeDirection ---
	degToMove := shortestPathToZero(driverStatus.currentPosition, envSettings.HomeDirection)
	logger.Debug("zero ref degToMove (RS232 style):", degToMove)

	// No configureDriver() here (your comment is correct: unsafe after activate)

	// --- PDO ONLY ---
	if !(IsPDOActive() && device.PdoPosReady) {
		return fmt.Errorf("PDO position not ready (PdoPosReady=false). Zero reference is PDO-only in this build")
	}

	_, declampErr := hasDeclamped(device, envSettings)
	if declampErr != nil {
		logger.Error(declampErr)
		return declampErr
	}
	notifyDriverStatus("motor_running", "true", device)

	// Force direction by converting "degToMove" into a relative pulse target:
	// targetPulse = currentPulse + (degToMove * DriveXRatio)
	currentPulse := device.PDOPos.Load() // Phase 4: read from this device's own PDO state
	deltaPulse := int32(float64(device.Device.DriveXRatio) * degToMove)
	targetPulse := currentPulse + deltaPulse

	// Use fixed zeroRefFeed constant — NOT envSettings.JogFeed.
	// This keeps zero-ref speed constant regardless of what the operator
	// sets for manual jog speed.
	logger.Info("[PDO-ZERO] setting homing speed. zeroRefFeed:", zeroRefFeed)
	if err := setRpm(device, zeroRefFeed); err != nil {
		logger.Warn("[PDO-ZERO] Could not set homing speed:", err)
	}

	// Sync ramp start
	device.currentTargetPosition.Store(currentPulse)
	_ = device.SetTargetPositionPDO(targetPulse)
	device.EnablePosPDO(true)

	logger.Info("[PDO-ZERO] CSP ramp to zero via relative pulse target. current:", currentPulse, "target:", targetPulse)

	if err := hasTargetReached(device); err != nil {
		device.EnablePosPDO(false)
		notifyDriverStatus("motor_running", "false", device)
		return err
	}

	device.EnablePosPDO(false)
	notifyDriverStatus("motor_running", "false", device)

	aposAtZero := device.PDOPos.Load()
	if err := settings.SaveHomingReference(device.Device.Name, aposAtZero); err != nil {
		logger.Error("[PDO-ZERO] SaveHomingReference failed:", err)
	} else {
		logger.Info("HomingApos saved after zero ref:", aposAtZero, "for driver:", device.Device.Name)
	}

	_, clampErr := hasClamped(device, envSettings)
	if clampErr != nil {
		logger.Error(clampErr)
		return clampErr
	}

	channels.DestinationReached()
	notifier.SocketMessage("gotozero_done", "goto zero completed")
	sendECSFinSignal(device)
	return nil
}

// shortestPathToZero returns the signed angular delta to reach 0° along the shortest arc.
func shortestPathToZero(currentPosition float64, homeDirection int) float64 {
	cwDeg := getPos(currentPosition, 0, true)
	ccwDeg := getPos(currentPosition, 0, false)
	switch {
	case math.Abs(cwDeg) < math.Abs(ccwDeg):
		return cwDeg
	case math.Abs(ccwDeg) < math.Abs(cwDeg):
		return ccwDeg
	default:
		if homeDirection == 1 {
			return cwDeg
		}
		return ccwDeg
	}
}

// getPos returns the angular distance to travel from currentPos to targetPos.
// If clockwise is true, the result is the clockwise arc; otherwise counter-clockwise.
func getPos(currentPos float64, targetPos float64, clockwise bool) float64 {
	currentPos = math.Mod(currentPos, 360)
	modeDiff := math.Mod((currentPos - targetPos), 360)

	if clockwise {
		return math.Mod((360 - modeDiff), 360)
	}
	return math.Mod((modeDiff * -1), 360)
}