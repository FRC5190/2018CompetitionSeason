/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.*
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.drive.*
import frc.team5190.robot.elevator.*
import frc.team5190.robot.intake.*
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {

        /**
         * Returns an autonomous command pertaining to the FMS data
         * @param startingPositions Starting Position
         * @param switchOwnedSide The owned side of the switch
         * @param scaleOwnedSide The owned side of the scale
         */
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, settings: Array<String>): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            return when (folder) {
                "CS-L", "CS-R" -> commandGroup {

                    val mpCommand = MotionProfileCommand(folder, "Switch", false, false)

                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(mpCommand.mpTime - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                        })
                    })

                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(StraightDriveCommand(-1.0), 0.7)

                    addSequential(commandGroup {
                        addParallel(TurnCommand(if (folder[folder.length - 1] == 'L') 90.0 else -90.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                    })

                    addSequential(PickupCubeCommand())
                    addSequential(IntakeHoldCommand(), 0.001)

                    addSequential(StraightDriveCommand(driveToZero = true))

                    addSequential(commandGroup {
                        addParallel(TurnCommand(0.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                    })

                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                }

                "LS-LL", "RS-RR", "LS-RL", "RS-LR",
                "LS-RR", "RS-LL", "LS-LR", "RS-RL"-> commandGroup {

                    val mpCommand = MotionProfileCommand(folder, "Scale", true, folder.first() == 'R')
                    val thirdcube = MotionProfileCommand(folder, "3rd Cube", true, folder.first() == 'R')

                    // DROP CUBE ON SCALE
                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.2))
                            addSequential(commandGroup {
                                addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                addParallel(AutoArmCommand(ArmPosition.UP))
                            })
                            addSequential(TimedCommand((mpCommand.mpTime - 3.75).coerceAtLeast(0.001)))
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                                addParallel(commandGroup {
                                    addSequential(TimedCommand(1.75))
                                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 0.5))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    // PICKUP SECOND CUBE
                    addSequential(commandGroup {
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ElevatorSubsystem.currentPosition < ElevatorPosition.FIRST_STAGE.ticks + 100.0
                            })
                            addSequential(PickupCubeCommand())
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // DROP SECOND CUBE IN SCALE
                    addSequential(commandGroup {
                        addParallel(ArcDriveCommand(-5.0, if (folder.first() == 'R') -12.5 else 12.5))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(1.75))
                            addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // PICKUP THIRD CUBE
                    addSequential(commandGroup {
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ElevatorSubsystem.currentPosition < ElevatorPosition.FIRST_STAGE.ticks + 100.0
                            })
                            addSequential(PickupCubeCommand())
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // DROP THIRD CUBE IN SCALE
                    addSequential(commandGroup {
                        addParallel(thirdcube)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.5))
                            addSequential(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                        })
                        addParallel(commandGroup {
                            addSequential(TimedCommand(2.25))
                            addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                }

                else -> throw IllegalArgumentException("Scenario does not exist.")
            }
        }
    }
}

/**
 * Stores starting position of the robot.
 */
enum class StartingPositions {
    LEFT, CENTER, RIGHT;
}
