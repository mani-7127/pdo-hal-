package motordriver

import (
	channels "EtherCAT/channels"
	logger "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	statusnotifier "EtherCAT/motordriver/statusnotifier"
	"errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"time"

	"github.com/google/uuid"
)

// ----------------------------------------------------------
// Precision configuration
// ----------------------------------------------------------
const (
	positionToleranceFastPath = 0.02 // degrees — skip motion if already within this
	positionToleranceFinal    = 0.02 // degrees — finish-signal gate
)

// motionMu prevents two moveMotorToDegree goroutines from running concurrently.
// Without this guard, two rapid MOVE_TO_POSITION messages spawn two goroutines
// that both pass the ECS-HIGH poll, both complete their settle loops, and both
// call sendECSFinSignal — causing the double finish-signal bug.
var motionMu sync.Mutex

// lastNormalFinMu guards the fields below, which record the destination and
// wall-clock time of the most recently sent fin signal on the NORMAL (non
// fast-path) completion code path.
//
// Why we need this:
//
//	The executor has a rare timing bug where it re-issues the command it just
//	completed (same line, same destination) before its internal line counter
//	advances.  When that duplicate arrives, the motor is already at the target
//	and the "already at target" fast path fires, sending a SECOND fin pulse.
//	The external machine counts each rising edge as a separate "move done"
//	event and steps its state machine twice, losing synchronisation.
//
//	Fix: after every normal-path move we stamp the destination and the time.
//	If the fast path is triggered for that SAME destination within a short
//	guard window (dupFinWindow), it is treated as the duplicate and the fin
//	signal is suppressed.  The executor is still unblocked via doneDriverAction
//	so program flow continues correctly.
var (
	lastNormalFinMu   sync.Mutex
	lastNormalFinDest float64
	lastNormalFinAt   time.Time
)

// dupFinWindow is the maximum gap between a completed normal move and a
// subsequent "already at target" fast-path call to the SAME destination that
// we consider a duplicate.  Chosen to be comfortably longer than the observed
// ~1 s anomaly window but shorter than the ~10 s loop period.
const dupFinWindow = 3 * time.Second

// ----------------------------------------------------------
// Helpers
// ----------------------------------------------------------

func clearTargetReached(device *MasterDevice) {
	logger.Debug("Clearing 'target reached' status for device:", device.Name)
}

func sleepMs(ms int) {
	time.Sleep(time.Duration(ms) * time.Millisecond)
}

// ----------------------------------------------------------
// Main Motion Function
// ----------------------------------------------------------

func moveMotorToDegree(device *MasterDevice, degreeToRotate float64) error {
	// Prevent concurrent executions from each sending a finish signal.
	// If a move is already in progress (e.g. duplicate channel message or
	// rapid successive commands), block here until the active move finishes.
	motionMu.Lock()
	defer motionMu.Unlock()

	driverStatus := getCurrentDriverStatus(device.Device.Name)
	// FIX: Bypass the asynchronous Go channel cache.
	// Read true position synchronously from the EtherCAT buffer to prevent skipped lines.
	truePos := ReadActualPositionFromDrive(device.Name)
	driverStatus.currentPosition = truePos
	if driverStatus.potNotExceeded {
		logger.Error("pot/not exceeded, exiting from move command")
		return errors.New("pot/not exceeded, exiting from move command")
	}

	uid := uuid.New()
	logger.Info("rotate motor to", degreeToRotate,
		"current:", driverStatus.currentPosition,
		"prev dest:", driverStatus.destinationPosition,
		"workoffset:", driverStatus.workOffset,
		"id:", uid,
	)

	var moveToPos, destination float64

	if driverStatus.mode == "ABS" {
		moveToPos, destination = getAbsolutePosition(
			driverStatus.currentPosition,
			degreeToRotate,
			driverStatus.shortestPathEnabled,
		)
	} else {
		moveToPos, destination = getRelativePosition(
			driverStatus.currentPosition,
			degreeToRotate,
			driverStatus.destinationPosition,
		)
	}

	notifier.NotifyDestinationPosition(device.Name, float32(destination-driverStatus.workOffset))

	// ----------------------------------------------------------
	// Wait for ECS
	// ----------------------------------------------------------
	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcs := doECSCheck(device, destination)
	if gotEcs == 0 || gotEcs == 2 {
		return fmt.Errorf("failed to receive ECS — program stopped or reset")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	// Refresh status after ECS latency
	driverStatusAfterECS := getCurrentDriverStatus(device.Device.Name)
	// FIX: Ensure the post-ECS status also uses the true hardware position
	driverStatusAfterECS.currentPosition = ReadActualPositionFromDrive(device.Name)

	// ----------------------------------------------------------
	// ABS fast-path: already at target, no motion needed
	// ----------------------------------------------------------
	if driverStatusAfterECS.mode == "ABS" &&
		math.Abs(driverStatusAfterECS.currentPosition-destination) <= positionToleranceFastPath {

		// Double-check to guard against a stale cache read
		//recheck := getCurrentDriverStatus(device.Device.Name)
		// FIX: Check against hardware, not the async cache again
		trueRecheckPos := ReadActualPositionFromDrive(device.Name)
		if math.Abs(trueRecheckPos-destination) <= positionToleranceFastPath {

			// ----------------------------------------------------------
			// Deduplication guard (double-fin prevention)
			// ----------------------------------------------------------
			// If the executor re-issues the same command it just completed
			// (a known rare timing/race in the executor), the motor is still
			// at the target and we would send a second fin pulse.  Check
			// whether a NORMAL-PATH fin was sent for this exact destination
			// within dupFinWindow and, if so, suppress the duplicate.
			lastNormalFinMu.Lock()
			recentSameDest := math.Abs(lastNormalFinDest-destination) <= positionToleranceFinal &&
				time.Since(lastNormalFinAt) < dupFinWindow
			lastNormalFinMu.Unlock()

			if recentSameDest {
				logger.Warn("[DEDUP] 'Already at target' for same destination as just-completed move — "+
					"suppressing duplicate fin signal. dest:", destination,
					"lastNormalFinDest:", lastNormalFinDest,
					"age:", time.Since(lastNormalFinAt).Round(time.Millisecond),
					"id:", uid)
				doneDriverAction()
				channels.DestinationReached()
				return nil
			}

			logger.Info("Already at target — sending finish immediately, id:", uid)
			sendECSFinSignal(device)

			channels.WriteCommandExecInput("waiting_for_ecs", "")
			if z := doECSCheckZero(device, destination); z == 0 || z == 2 {
				return fmt.Errorf("failed to receive ECS zero")
			}
			channels.WriteCommandExecInput("ecs_done", "")

			doneDriverAction()
			channels.DestinationReached()
			return nil
		}
	}

	// ----------------------------------------------------------
	// Set direction and store destination
	// ----------------------------------------------------------
	setDirection(device, driverStatusAfterECS, moveToPos)
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", destination), device)

	// ----------------------------------------------------------
	// Apply pitch error and backlash compensation
	// ----------------------------------------------------------
	backlash := driverStatusAfterECS.backlash
	pitchErr := getPitchError(device.Name, destination)
	moveToWithComp := moveToPos + pitchErr - backlash

	clearTargetReached(device)

	// ----------------------------------------------------------
	// Execute rotation — retry up to 3 times if drive misses the set-point.
	//
	// WHY: The drive occasionally ACKs the set-point (bit12=1 clears ppSetpointPending)
	// but physically never starts moving (bit10 never clears). hasTargetReached Phase 1.5
	// now detects this and returns a recoverable error. Re-arming ppSetpointPending and
	// retrying the PDO goal write recovers cleanly without stopping the program.
	// ----------------------------------------------------------
	const maxRotateRetries = 3
	var rotErr error
	for attempt := 0; attempt < maxRotateRetries; attempt++ {
		if attempt > 0 {
			logger.Warn("[RETRY] doRotate attempt", attempt+1, "of", maxRotateRetries,
				"— re-arming set-point. Previous error:", rotErr)
			// Re-arm the PP set-point handshake so the cyclic task re-sends bit4 HIGH.
			device.ppSetpointPending.Store(true)
			time.Sleep(50 * time.Millisecond)
		}
		rotErr = doRotate(device, moveToWithComp)
		if rotErr == nil {
			break
		}
		// Only retry on "motor did not start" / "false target reached" — not on faults,
		// timeouts, or PDO-stopped errors, which require operator intervention.
		if !strings.Contains(rotErr.Error(), "did not start") &&
			!strings.Contains(rotErr.Error(), "false target reached") {
			break
		}
	}
	if rotErr != nil {
		errMsg := fmt.Sprintf("[PDO-PP] move failed after %d attempts: %v", maxRotateRetries, rotErr)
		logger.Error(errMsg)
		statusnotifier.Alarm(errMsg)
		// CRITICAL: call doneDriverAction so the executor does NOT hang forever on
		// WaitExecuteNextCommand(). Without this, a failed move blocks the program
		// indefinitely — the user would have to manually click Stop every time.
		doneDriverAction()
		return rotErr
	}

	// NOTE: No settle loop here.
	// hasTargetReached() inside doRotate already confirmed bit10=1 AND bit12=0
	// stable for 50 consecutive 1ms PDO cycles before returning. Re-polling
	// position + bit10 again would add up to 20ms of unnecessary latency on
	// every move with zero safety benefit.

	finalPos := ReadActualPositionFromDrive(device.Name)

	// Calculate raw absolute difference
	errVal := math.Abs(finalPos - destination)

	// Account for 360-degree wrap-around
	errVal = math.Mod(errVal, 360.0)
	if errVal > 180.0 {
		errVal = 360.0 - errVal
	}

	if errVal > positionToleranceFinal {
		errMsg := fmt.Sprintf("FINISH BLOCKED — final position outside tolerance target: %g current: %g error: %g",
			destination, finalPos, errVal)
		logger.Error(errMsg)
		// Show the error on the UI so the operator knows why the program stopped.
		statusnotifier.Alarm(errMsg)
		// CRITICAL: unblock the executor. Without doneDriverAction() here, the executor
		// is stuck forever waiting on WaitExecuteNextCommand() — the program appears
		// frozen between two lines until the user manually clicks Stop.
		doneDriverAction()
		return fmt.Errorf("finish blocked: target not reached (error=%.4f°)", errVal)
	}

	if IsPDOActive() && device.PdoPosReady {
		logger.Info("[PDO-PP] position AND bit10 confirmed, sending finish signal. id:", uid)
	}

	// ----------------------------------------------------------
	// Send finish signal and stamp the deduplication tracker
	// ----------------------------------------------------------
	sendECSFinSignal(device)

	// Record this destination + time so the "already at target" fast path can
	// detect a rapid duplicate command from the executor and suppress the
	// spurious second fin pulse (see dupFinWindow / lastNormalFin vars above).
	lastNormalFinMu.Lock()
	lastNormalFinDest = destination
	lastNormalFinAt = time.Now()
	lastNormalFinMu.Unlock()

	// ----------------------------------------------------------
	// Wait for ECS zero
	// ----------------------------------------------------------
	channels.WriteCommandExecInput("waiting_for_ecs", "")
	if z := doECSCheckZero(device, destination); z == 0 || z == 2 {
		return fmt.Errorf("failed to receive ECS zero")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	doneDriverAction()
	channels.DestinationReached()
	logger.Info("move to position completed, driver:", device.Name, "id:", uid)
	return nil
}

// ----------------------------------------------------------
// Step Mode
// ----------------------------------------------------------

func stepMode(masterDevice *MasterDevice, valueInDegreeToAdd float64) error {
	logger.Debug("step mode moving to position:", valueInDegreeToAdd)

	driverStatus := getCurrentDriverStatus(masterDevice.Device.Name)
	if driverStatus.potNotExceeded {
		logger.Error("pot/not exceeded, exiting from move command")
		channels.StepModeComplete()
		return errors.New("pot/not exceeded, exiting from move command")
	}

	err := freeRotate(masterDevice, valueInDegreeToAdd)
	channels.StepModeComplete()
	return err
}

// ----------------------------------------------------------
// Direction Selection
// ----------------------------------------------------------

func setDirection(device *MasterDevice, driverStatus driverCurrentStatus, degreeToRotate float64) {
	if !driverStatus.shortestPathEnabled {
		notifyDriverStatusWithWait("rotation_direction", "1", device)
		return
	}
	if degreeToRotate > 0 {
		notifyDriverStatusWithWait("rotation_direction", "1", device)
	} else {
		notifyDriverStatusWithWait("rotation_direction", "-1", device)
	}
}

// ----------------------------------------------------------
// Position Sync Helpers
// ----------------------------------------------------------

// RefreshCurrentPosition reads the actual encoder position from every drive
// and updates the internal status cache for each.
//
// FIX (multi-axis): the old version hard-coded devices[0], so after a PP move
// on Axis B the cached position for Axis A was refreshed instead — leaving the
// executor's "current position" stale for all axes except the first.
// Now all configured devices are refreshed in the same call.
func RefreshCurrentPosition() {
	devices := getMasterDevices()
	if len(devices) == 0 {
		logger.Warn("[SYNC] RefreshCurrentPosition: no master devices available")
		return
	}
	for _, dev := range devices {
		var rawPulses int32
		if dev.PdoReady {
			rawPulses = dev.PDOPos.Load()
		} else {
			// SDO fallback: read 0x6064 directly
			operation, opErr := GetEtherCATOperation("pollstatus", dev.Device.AddressConfigName)
			if opErr != nil || len(operation.Steps) == 0 {
				logger.Error("[SYNC] RefreshCurrentPosition: could not get pollstatus operation:", opErr)
				continue
			}
			val, sdoErr := DrivePosition(dev.Master, dev.Position, operation.Steps[0])
			if sdoErr != nil {
				logger.Error("[SYNC] RefreshCurrentPosition: SDO read failed:", sdoErr)
				continue
			}
			rawPulses = val
		}

		degrees, _ := currentPosition(rawPulses, dev.Device.DriveXRatio, dev.Name)
		currentDriverPosition(dev, degrees)
		logger.Info(fmt.Sprintf("[SYNC] Position refreshed to %.3f° (raw: %d) for drive %s",
			degrees, rawPulses, dev.Name))
	}
}

// ReadActualPositionFromDrive returns the current position in degrees directly
// from the PDO buffer (or SDO on fallback), NOT from the cached status keeper.
func ReadActualPositionFromDrive(driveName string) float64 {
	devices := getMasterDevices()
	for _, dev := range devices {
		if dev.Name == driveName {
			var rawPulses int32
			if dev.PdoReady {
				rawPulses = dev.PDOPos.Load()
			} else {
				operation, opErr := GetEtherCATOperation("pollstatus", dev.Device.AddressConfigName)
				if opErr != nil || len(operation.Steps) == 0 {
					break
				}
				val, sdoErr := DrivePosition(dev.Master, dev.Position, operation.Steps[0])
				if sdoErr != nil {
					break
				}
				rawPulses = val
			}
			degrees, _ := currentPosition(rawPulses, dev.Device.DriveXRatio, driveName)
			return degrees
		}
	}
	// Last resort: return cached value
	return getCurrentDriverStatus(driveName).currentPosition
}
