/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.drive.StraightDriveCommand
import frc.team5190.robot.elevator.ElevatorPreset
import frc.team5190.robot.elevator.ElevatorPresetCommand
import frc.team5190.robot.util.commandGroup

class ClosedLoopDiagnostics : CommandGroup() {

    private val straightDriveCommand = StraightDriveCommand(3.0)
    private val elevatorBehind = ElevatorPresetCommand(ElevatorPreset.BEHIND)
    private val elevatorDown = ElevatorPresetCommand(ElevatorPreset.INTAKE)

    private var startTime = -1L

    init {
        addParallel(straightDriveCommand)
        addParallel(commandGroup {
            addSequential(elevatorBehind)
            addSequential(elevatorDown)
        })
    }

    override fun initialize() {
        super.initialize()
        startTime = System.currentTimeMillis()
    }

    override fun end() {
        if (straightDriveCommand.isCompleted && elevatorBehind.isCompleted && elevatorDown.isCompleted) {
            println("Closed Loop OK")
        } else {
            println("Closed Loop FAILED")
        }
    }

    override fun isFinished() = System.currentTimeMillis() - startTime > 6000
}