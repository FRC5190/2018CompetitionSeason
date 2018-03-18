/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
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
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            return when (folder) {
                "CS-L", "CS-R" -> commandGroup {

                    val mpCommand = MotionProfileCommand(folder, "Switch", false, false)

                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(AutoArmCommand(ArmPosition.UP))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(mpCommand.mpTime - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                        })
                    })

                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(StraightDriveCommand(-1.0), 0.7)

                    addSequential(commandGroup {
                        addParallel(TurnCommand(if (folder.last() == 'L') 70.0 else -70.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE), 2.0)
                    })

                    addSequential(PickupCubeCommand(), 2.0)
                    addSequential(IntakeHoldCommand(), 0.001)

                    addSequential(StraightDriveCommand(driveToZero = true))

                    addSequential(commandGroup {
                        addParallel(TurnCommand(0.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                    })

                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                }

                "LS-LL", "RS-RR", "LS-RL", "RS-LR",
                "LS-RR", "RS-LL", "LS-LR", "RS-RL" -> commandGroup {

                    val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"
                    val mpCommand = MotionProfileCommand(folderIn, "Scale", true, folder.first() == 'R')

                    // DROP CUBE ON SCALE
                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.2))
                            addSequential(commandGroup {
                                addSequential(object : CommandGroup() {
                                    var startTime: Long = 0

                                    init {
                                        addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        addParallel(AutoArmCommand(ArmPosition.UP))
                                    }

                                    override fun initialize() {
                                        super.initialize()
                                        startTime = System.currentTimeMillis()
                                    }

                                    override fun isFinished() = (System.currentTimeMillis() - startTime) > (mpCommand.mpTime - 2.50).coerceAtLeast(0.001) * 1000
                                })
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                                addParallel(commandGroup {
                                    addSequential(TimedCommand(1.95))
                                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.75, timeout = 1.0))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    // PICKUP SECOND CUBE
                    addSequential(commandGroup {
                        addSequential(commandGroup {
                            addParallel(TurnCommand(if (folder.last() == 'R') 7.5 else -7.5))
                            addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        })
                        addSequential(PickupCubeCommand(inSpeed = 1.0))
                        addSequential(IntakeHoldCommand(), 0.001)
                    })


                    // DROP SECOND CUBE IN SCALE
                    addSequential(commandGroup {
                        addParallel(ArcDriveCommand(-5.0, if (folder.first() == 'R') -12.5 else 12.5), 4.0)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(2.15))
                            addSequential(object : IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 0.5) {
                                override fun end() {
                                    DriveSubsystem.currentCommand?.cancel()
                                }
                            })
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    addSequential(ElevatorPresetCommand(ElevatorPreset.INTAKE))
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
