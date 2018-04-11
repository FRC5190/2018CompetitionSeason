/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.drive.*
import frc.team5190.robot.elevator.*
import frc.team5190.robot.intake.IntakeCommand
import frc.team5190.robot.intake.IntakeDirection
import frc.team5190.robot.intake.IntakeHoldCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    object ModernAuto {
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, cubes: Int, modernCrossAuto: Boolean): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            return when (folder) {
                "CS-L", "CS-R" -> LegacyAuto.getAuto(startingPositions, switchOwnedSide, scaleOwnedSide)

                "LS-LL", "RS-RR", "LS-RL", "RS-LR" -> commandGroup {

                    val timeToGoUp = if (folder.first() == folder.last()) 2.50 else 1.50
                    val firstCube = MotionProfileCommand(if (folder.first() == folder.last()) "LS-LL" else "LS-RR", "Drop First Cube",
                            robotReversed = true, pathMirrored = folder.first() == 'R')

                    /*
                    Drop 1st Cube in Scale
                    */
                    addSequential(commandGroup {
                        addParallel(firstCube)
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

                                override fun isFinished() = (System.currentTimeMillis() - startTime) > (firstCube.pathDuration - timeToGoUp).coerceAtLeast(0.001) * 1000
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR))
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    })
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.40, timeout = 0.50))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    /*
                    Pickup 2nd Cube
                     */
                    addSequential(commandGroup {
                        addSequential(commandGroup {
                            addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                            addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                            addParallel(object : MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = folder.last() == 'R') {
                                override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                            })
                        })
                        addSequential(IntakeHoldCommand(), 0.001)

                    })

                    /*
                     Drop 2nd Cube in Scale
                      */
                    addSequential(commandGroup {
                        val dropSecondCubePath = MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = folder.last() == 'R')
                        addParallel(dropSecondCubePath)
                        addParallel(commandGroup {
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    })
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.50, timeout = 0.50))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    if (cubes > 2) {
                        /*
                    Pickup 3rd Cube
                     */
                        addSequential(commandGroup {
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                                addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                                addParallel(object : MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = folder.last() == 'R') {
                                    override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                                })
                            })
                            addSequential(IntakeHoldCommand(), 0.001)
                        })

                        /*
                         Go to Scale with 3rd Cube
                          */
                        addSequential(commandGroup {
                            val dropThirdCubePath = MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = folder.last() == 'R')
                            addParallel(dropThirdCubePath)
                            addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                        })
                    }
                }
                "LS-RR", "RS-LL", "LS-LR", "RS-RL" -> if (!modernCrossAuto) LegacyAuto.getAuto(startingPositions, switchOwnedSide, scaleOwnedSide) else commandGroup {

                    val timeToGoUp = if (folder.first() == folder.last()) 2.50 else 1.50
                    val firstCube = MotionProfileCommand(if (folder.first() == folder.last()) "LS-LL" else "LS-RR", "Drop First Cube",
                            robotReversed = true, pathMirrored = folder.first() == 'R')

                    /*
                    Drop 1st Cube in Scale
                    */
                    addSequential(commandGroup {
                        addParallel(firstCube)
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

                                override fun isFinished() = (System.currentTimeMillis() - startTime) > (firstCube.pathDuration - timeToGoUp).coerceAtLeast(0.001) * 1000
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR))
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    })
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.40, timeout = 0.50))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    /*
                    Pickup 2nd Cube
                     */
                    addSequential(commandGroup {
                        addSequential(commandGroup {
                            addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                            addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                            addParallel(object : MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = folder.last() == 'R') {
                                override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                            })
                        })
                        addSequential(IntakeHoldCommand(), 0.001)

                    })

                    /*
                     Drop 2nd Cube in Scale
                      */
                    addSequential(commandGroup {
                        val dropSecondCubePath = MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = folder.last() == 'R')
                        addParallel(dropSecondCubePath)
                        addParallel(commandGroup {
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    })
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.50, timeout = 0.50))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })

                    if (cubes > 2) {
                        /*
                    Pickup 3rd Cube
                     */
                        addSequential(commandGroup {
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                                addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                                addParallel(object : MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = folder.last() == 'R') {
                                    override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                                })
                            })
                            addSequential(IntakeHoldCommand(), 0.001)
                        })

                        /*
                         Go to Scale with 3rd Cube
                          */
                        addSequential(commandGroup {
                            val dropThirdCubePath = MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = folder.last() == 'R')
                            addParallel(dropThirdCubePath)
                            addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                        })
                    }
                }

                else -> commandGroup { addSequential(StraightDriveCommand((if (startingPositions == StartingPositions.CENTER) 1 else -1) * 10.0)) }
            }
        }
    }

    object LegacyAuto {
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            return when (folder) {
            // Center switch autonomous cases.
                "CS-L", "CS-R" -> commandGroup {
                    val firstSwitch = MotionProfileCommand(folder, "Switch", false, false)

                    addSequential(commandGroup {
                        addParallel(firstSwitch)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH), 3.0)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(firstSwitch.pathDuration - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, speed = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                    addSequential(commandGroup {
                        addParallel(MotionProfileCommand(folder, "Center", robotReversed = true, pathReversed = true))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.5))
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                            })
                        })
                    })
                    addSequential(TurnCommand(angle = if (folder.last() == 'L') -5.0 else 0.0))
                    addSequential(PickupCubeCommand(visionCheck = false), 4.0)
                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(ArcDriveCommand(-5.0, angle = 0.0, cruiseVel = 5.0, accel = 4.0), 1.75)

                    addSequential(commandGroup {
                        addParallel(MotionProfileCommand(folder, "Switch", false, false), firstSwitch.pathDuration - 0.4)
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                        addParallel(commandGroup {
                            addSequential(TimedCommand(firstSwitch.pathDuration - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, speed = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                }

            // Scale autonomous cases
                "LS-LL", "RS-RR", "LS-RL", "RS-LR" -> commandGroup {

                    val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"
                    val timeToGoUp = if (folder.first() == folder.last()) 2.50 else 1.50
                    val mpCommand = MotionProfileCommand(folderIn, "Scale", true, false, folder.first() == 'R')

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

                                override fun isFinished() = (System.currentTimeMillis() - startTime) > (mpCommand.pathDuration - timeToGoUp).coerceAtLeast(0.001) * 1000
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND), 3.0)
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    }, 3.0)
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = if (folder.first() == folder.last()) 0.85 else 0.75, timeout = 1.0))
                                    addSequential(IntakeHoldCommand(), 0.001)
                                })
                            })
                        })
                    })
                }
                "LS-RR", "RS-LL", "LS-LR", "RS-RL" -> commandGroup {

                    val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"
                    val timeToGoUp = if (folder.first() == folder.last()) 2.50 else 1.50
                    val mpCommand = MotionProfileCommand(folderIn, "Scale", true, false, folder.first() == 'R')

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

                                override fun isFinished() = (System.currentTimeMillis() - startTime) > (mpCommand.pathDuration - timeToGoUp).coerceAtLeast(0.001) * 1000
                            })
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                                addParallel(commandGroup {
                                    addSequential(object : Command() {
                                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                                    }, 3.0)
                                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = if (folder.first() == folder.last()) 0.85 else 0.75, timeout = 1.0))
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
                            addSequential(TurnCommand(180.0))
                            addSequential(object : Command() {
                                override fun isFinished() = ElevatorSubsystem.currentPosition < ElevatorPosition.SWITCH.ticks - 100
                            })
                            addSequential(PickupCubeCommand(inSpeed = -1.0, visionCheck = folder.first() == folder.last()), 3.25)
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // Drop 2nd Cube in Scale
                    addSequential(commandGroup {
                        addParallel(ArcDriveCommand(-5.75, -179.0, 4.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR))
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(TimedCommand(0.2))
                            addSequential(object : IntakeCommand(IntakeDirection.OUT, speed = 0.75, timeout = 0.95) {
                                override fun end() {
                                    DriveSubsystem.currentCommand?.cancel()
                                }
                            })
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    // TODO POTENTIAL 3RD CUBE TESTING FOR ASHEVILLE
                    addSequential(commandGroup {
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        addParallel(commandGroup {
                            addSequential(StraightDriveCommand(0.1), 0.01)
                            addSequential(TurnCommand(if (folder.last() == 'R') -20.0 else 20.0))
                            addSequential(object : Command() {
                                override fun isFinished() = ElevatorSubsystem.currentPosition < ElevatorPosition.SWITCH.ticks - 100
                            })
                            addSequential(PickupCubeCommand(inSpeed = -1.0), 3.25)
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
