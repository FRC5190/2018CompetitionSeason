/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.*
import frc.team5190.robot.arm.*
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
            // Center switch autonomous cases.
                "CS-L", "CS-R" -> commandGroup {
                    val firstSwitch = MotionProfileCommand(folder, "Switch", false, false)
                    val secondSwitch = MotionProfileCommand(folder, "Switch 2", false, false)

                    addSequential(commandGroup {
                        addParallel(firstSwitch)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(firstSwitch.mpTime - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                    addSequential(commandGroup {
                        addParallel(MotionProfileCommand(folder, "Center", true, false))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.5))
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                            })
                        })
                    })
                    addSequential(TurnCommand(0.0, false, 0.0))
                    addSequential(commandGroup {
                        addParallel(StraightDriveCommand(4.50))
                        addParallel(IntakeCommand(IntakeDirection.IN, timeout = 4.0))
                    })
                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(StraightDriveCommand(-4.25, cruiseVel = 5.0, accel = 4.0), 1.2)

                    addSequential(commandGroup {
                        addParallel(secondSwitch)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(secondSwitch.mpTime - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                }



            // Scale autonomous cases
                "LS-LL", "RS-RR", "LS-RL", "RS-LR",
                "LS-RR", "RS-LL", "LS-LR", "RS-RL" -> commandGroup {

                    val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"
                    val timeToGoUp = if (folder.first() == folder.last()) 2.50 else 1.50
                    val mpCommand = MotionProfileCommand(folderIn, "Scale", true, folder.first() == 'R')

                    // Drop 1st Cube on Scale
                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.2))
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

                                override fun isFinished() = (System.currentTimeMillis() - startTime) > (mpCommand.mpTime - timeToGoUp).coerceAtLeast(0.001) * 1000
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    })
                                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = if (folder.first() == folder.last()) 1.0 else 0.75, timeout = 1.0))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    // Pickup 2nd Cube
                    addSequential(commandGroup {
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        addParallel(commandGroup {
                            addSequential(StraightDriveCommand(0.1), 0.01)
                            addSequential(TurnCommand(if (folder.last() == 'R') 8.0 else -10.0))
                            addSequential(object : Command() {
                                override fun isFinished() = ElevatorSubsystem.currentPosition < ElevatorPosition.SWITCH.ticks - 100
                            })
                            addSequential(PickupCubeCommand(inSpeed = -1.0), 3.25)
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // Drop 2nd Cube in Scale
                    addSequential(commandGroup {
                        addParallel(ArcDriveCommand(-5.0, if (folder.last() == 'R') -12.5 else 12.5), 4.0)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND))
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(TimedCommand(0.25))
                            addSequential(object : IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 0.75) {
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
