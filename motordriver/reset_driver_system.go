package motordriver

import (
	channels "EtherCAT/channels"
	"EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"fmt"
	"time"
)
/*
resetSystemWorker is the single goroutine that handles all system resets.
It reads from channels.ResetDriverSystem and executes the full reset sequence.

RESET SEQUENCE — order is critical for IgH EtherCAT correctness:

  PHASE 1 — Stop motion and listeners (PDO cyclic still running)
    a. stopECSCheck             — unblock any goroutine waiting in ECS loop
    b. stopDriverPolling        — stop 50ms position-poll goroutines
    c. stopPollIOStat           — stop digital-input poll goroutines
    d. stopErrorPolling         — stop error-code poller (no-op when PDO active)
    e. stopDriverActionListener — signal action listener goroutine to exit
    f. stopDriveStatusListener  — signal status keeper goroutine to exit

  PHASE 2 — Guard: abort reset if PDO is not active (no safe SDO fallback)
    If IsPDOActive() == false →log error, re-start listeners, continue.

  PHASE 3 — Fault reset via PDO (cyclic MUST still be running)
    g. PDOFaultResetdis able jog/pos modes, clear drive fault via CiA-402
                       state machine. Cyclic is alive to process controlwords.
    h. Sleep 500ms  allow state machine to settle to Operation Enabled.

  PHASE 4 Restart listeners
    i. pollDrivePosition / pollIOStat / initDriverActionListener /
       startDriverStatusListener

  PDO IS MANDATORY THERE IS NO SDO FALLBACK AT RUNTIME:
    IgH EtherCAT has no ecrt_master_deactivate(). After ecrt_master_activate()
    the master owns the bus. SDO mailbox responses require ecrt_master_receive()
    to be running in the cyclic task. Stopping the cyclic before SDO calls
    causes those calls to block forever. If PDO failed to start during InitMaster,
    the process must be restarted no safe SDO recovery path exists at runtime.
*/

func listenSystemReset() {
	channels.ResetDriverSystem = make(chan bool)
	go resetSystemWorker()
}

func performSysReset(checkMotorRunning bool) {
	if checkMotorRunning {
		for _, dev := range masterDevices {
			stat := getCurrentDriverStatus(dev.Name)
			if stat.isMotorRunning {
				logger.Info("Unable to do system reset, motor", dev.Name, "is running")
				statusnotifier.Alarm("Unable to do system reset, motor " + dev.Name + " is running")
				statusnotifier.SocketMessage("reset_done", "reset completed")
				return
			}
		}
	}
	channels.ResetDriverSystem <- true
}

func resetSystemWorker() {
	for {
		msg := <-channels.ResetDriverSystem
		if msg == true {
			logger.Info("resetting driver system....")

			// Abort any in-flight position move BEFORE doing anything else.
			//
			// ROOT CAUSE (found via SD700 testing, but this is fully generic —
			// applies identically to A6B/Delta/Nidec): posMoveAborted was
			// previously only ever set by emergency() (drive_power.go). A plain
			// Reset never touched it. If a move's hasTargetReached() call was
			// still blocked waiting for bit10 (e.g. a slow homing move) when
			// Reset fired, that goroutine was NEVER signalled to stop — it kept
			// running orphaned for up to its own 30s timeout, still holding
			// motionMu and still polling/writing device state, concurrently
			// with whatever NEW move got issued right after the reset. This
			// produced exactly the "false target reached" / stale-diff symptom
			// (an old move's leftover state bleeding into a new one), and would
			// happen on any drive, not just SD700.
			//
			// Fix: set posMoveAborted for every device here too — same signal
			// emergency() already uses, so hasTargetReached's existing exit
			// path handles it identically for both cases.
			for _, dev := range masterDevices {
				dev.posMoveAborted.Store(true)
			}
			// hasTargetReached polls every 1ms — 20ms is ample time for any
			// truly in-flight move to observe the abort and exit. If nothing
			// was actually moving, this sleep just costs 20ms on every Reset.
			// Explicitly clearing the flag afterward (rather than relying on
			// hasTargetReached to consume it) matters: if NO move was in
			// flight, nothing would ever reset it back to false, and it would
			// incorrectly cancel the very next legitimate move on its first
			// Phase 2 check.
			time.Sleep(20 * time.Millisecond)
			for _, dev := range masterDevices {
				dev.posMoveAborted.Store(false)
			}

			// 1. Stop high-level listeners (aborts any active command sequences)
			stopECSCheck() // MUST be first: unblocks any goroutine waiting in doECSCheck / doECSCheckZero
			stopDriverPolling()
			stopDriverActionListener()
			stopDriveStatusListener()
			stopErrorPolling()
			stopPollIOStat()
			doneDriverAction()
			
			channels.WriteCommandExecInput("reset", "")
			channels.NotifyCmdComplete()

			// 2. CLEAR FAULTS AND HALT MOTION SAFELY
			//
			// PDO is the ONLY safe reset path at runtime. IgH EtherCAT has no
			// ecrt_master_deactivate(). SDO mailbox responses require ecrt_master_receive()
			// to be running in the cyclic task. Stopping the cyclic before SDO calls
			// causes those calls to block forever. If PDO failed to start, the process
			// must be restarted — no safe SDO fallback exists at runtime.
			if !IsPDOActive() {
				logger.Error("[RESET] FATAL: PDO cyclic is not active — cannot perform safe reset. Restart required.")
				statusnotifier.Alarm("Reset failed: EtherCAT PDO not active — restart required")
				statusnotifier.SocketMessage("reset_done", "reset failed")
				// Re-start listeners so the system stays responsive for a restart command.
				pollDrivePosition(masterDevices)
				pollIOStat(masterDevices)
				initDriverActionListener()
				startDriverStatusListener()
				continue // keep goroutine alive to handle future reset requests
			}

			logger.Info("[PDO] Performing soft reset via continuous PDO state machine...")
			// PDOFaultReset mirrors the YAML faultReset sequence:
			//   1. Halt motion (disable jog/pos, zero velocity)
			//   2. Issue fault reset (CW=0x80 via cia402 state machine)
			//   3. Wait up to 2s for fault bit to clear
			//   4. Release brake (0x60FE:01/02 = 0, 500ms delay)
			//   5. Wait for Operation Enabled state
			//   6. Clear cached error code (lastPDOErr = 0)
			// Returns false if the drive could not exit fault state
			// (non-resettable alarm, wrong controlword, or hardware E-stop active).
			resetOK := PDOFaultReset(masterDevices)

			// 3. RESTART LISTENERS
			// Always restart position, IO, action and status listeners — the UI
			// needs position updates and IO state regardless of fault status.
			pollDrivePosition(masterDevices)
			pollIOStat(masterDevices)
			initDriverActionListener()
			startDriverStatusListener()

			// FIX: Always restart the error poller regardless of whether the
			// reset succeeded.
			//
			// ROOT CAUSE: the previous conditional (if resetOK) permanently
			// stopped the error poller after a failed reset — for example when
			// Error 80 could not be cleared by PDOFaultReset. All subsequent
			// drive errors became invisible to the UI and logs until the
			// application was restarted, making the system appear to have
			// recovered when it had not.
			//
			// The concern about re-firing the same alarm immediately after a
			// failed reset is handled by the rising-edge detection inside
			// pollDriveErrWorker: it only calls DriverError() when errCode
			// CHANGES from lastReportedErrCode. The code that was already
			// reported before the reset attempt will NOT re-fire unless the
			// drive reports a new or different error code.
			pollDriveError(masterDevices)

			logger.Info("resetting driver completed....")

			if !resetOK {
				logger.Warn("[RESET] Drive still faulted after reset — error poller running for monitoring.")
				for _, dev := range masterDevices {
					logger.Warn(fmt.Sprintf("[RESET] %s sw=0x%04X", dev.Name, uint16(dev.PDOStatus.Load()&0xFFFF)))
				}
				statusnotifier.Alarm("Drive fault could not be cleared — check alarm code and try again")
				statusnotifier.SocketMessage("reset_done", "reset failed")
			} else {
				statusnotifier.Alarm("No Alarms")
				statusnotifier.SocketMessage("reset_done", "reset completed")
			}
		}
	}
}
// StopSystem stops all listeners and powers off drives cleanly.
func StopSystem() {
	if !HasDriverConnected() {
		return
	}
	stopECSCheck()
	stopDriverPolling()
	stopDriverActionListener()
	stopDriveStatusListener()
	stopErrorPolling()
	channels.WriteCommandExecInput("reset", "")
	channels.NotifyCmdComplete()
	// ShutdownMasters implements the 3-phase PDO shutdown:
	// arm pdoShutdownActive → poll PDS safe → StopPDOCyclic + ecrt_release_master.
	// This is the only correct way to stop without triggering Err88.2.
	ShutdownMasters()
}