package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.drive.ArcDriveCommand
import frc.team5190.robot.elevator.ElevatorPreset
import frc.team5190.robot.elevator.ElevatorPresetCommand
import frc.team5190.robot.intake.IntakeCommand
import frc.team5190.robot.intake.IntakeDirection
import frc.team5190.robot.intake.IntakeHoldCommand
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

class AutoHelper2 {
    companion object {
        fun getAuto(startingPositions: StartingPositions,
                    switchOwnedSide: MatchData.OwnedSide,
                    scaleOwnedSide: MatchData.OwnedSide,
                    sameSideAutoMode: AutoModes,
                    crossAutoMode: AutoModes): CommandGroup {

            when (startingPositions) {
                StartingPositions.CENTER -> {
                    return commandGroup {

                        val firstSwitch = MotionProfileCommand2(if (switchOwnedSide == MatchData.OwnedSide.LEFT) {
                            FastTrajectories.centerStartToLeftSwitch
                        } else {
                            FastTrajectories.centerStartToRightSwitch
                        })

                        val secondSwitch = MotionProfileCommand2(FastTrajectories.centerToSwitch, switchOwnedSide == MatchData.OwnedSide.RIGHT)

                        addSequential(commandGroup {
                            addParallel(firstSwitch) // Path to go to switch
                            addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH), 3.0) // Elevator to switch height
                            addParallel(commandGroup {
                                addSequential(TimedCommand(firstSwitch.pathDuration - 0.2)) // Wait 0.2 seconds before path ends
                                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, speed = 0.5)) // Outtake cube
                                addSequential(IntakeHoldCommand(), 0.001)
                            })
                        })
                        addSequential(commandGroup {
                            addParallel(MotionProfileCommand2(FastTrajectories.switchToCenter, mirrored = switchOwnedSide == MatchData.OwnedSide.RIGHT)) // Go back to center
                            addParallel(commandGroup {
                                addSequential(TimedCommand(0.5)) // Wait 0.5 seconds
                                addSequential(commandGroup {
                                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Get elevator to intaking position
                                })
                            })
                        })
                        addSequential(commandGroup {
                            addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0)) // pickup pyramid cube
                            addParallel(MotionProfileCommand2(FastTrajectories.centerToPyramid))
                        })

                        addSequential(IntakeHoldCommand(), 0.001)
                        addSequential(MotionProfileCommand2(FastTrajectories.pyramidToCenter)) // Drive back to wall

                        addSequential(commandGroup {
                            addParallel(secondSwitch) // Path to go to switch
                            addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH), 3.0) // Elevator to switch height
                            addParallel(commandGroup {
                                addSequential(TimedCommand(secondSwitch.pathDuration - 0.2)) // Wait 0.2 seconds before path ends
                                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, speed = 0.5)) // Outtake cube
                                addSequential(IntakeHoldCommand(), 0.001)
                            })
                        })
                    }
                }

                StartingPositions.RIGHT, StartingPositions.LEFT -> {

                }
            }
        }
    }
}