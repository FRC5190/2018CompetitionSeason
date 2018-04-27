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

class AutoHelper {
    companion object {
        // Returns a command group with the appropriate autonomous routine
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, sameSideAutoMode: AutoModes, crossAutoMode: AutoModes): CommandGroup {

            // Get the folder in which the paths are contained
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            // Other variables to determine if paths should be mirrored or reversed
            val isRightStart = folder.first() == 'R'
            val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"

            return when (folder) {
                // Switch auto from the center
                "CS-L", "CS-R" -> commandGroup {
                    val firstSwitch = MotionProfileCommand(folder, "Switch", false, false)

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
                        addParallel(MotionProfileCommand(folder, "Center", robotReversed = true, pathReversed = true)) // Go back to center
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.5)) // Wait 0.5 seconds
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Get elevator to intaking position
                            })
                        })
                    })
                    addSequential(TurnCommand(angle = if (folder.last() == 'L') -2.0 else 0.0), 1.0) // Turn to face cube
                    addSequential(PickupCubeCommand(visionCheck = false), 4.0) // Pickup cube
                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(ArcDriveCommand(-5.0, angle = 0.0, cruiseVel = 5.0, accel = 4.0), 1.75) // Drive back to wall 
                }

                // Same side scale auto from the sides
                "LS-LL", "LS-RL", "RS-RR", "RS-LR" -> when (sameSideAutoMode) {
                    // Full 3 cube auto
                    AutoModes.FULL -> getFullAuto(folderIn, isRightStart, scaleOwnedSide)

                    // 1 cube auto that does not interfere with other bots
                    AutoModes.SIMPLE -> getSimpleAuto(folderIn, isRightStart)

                    // Switch auto if it's on our side
                    AutoModes.SWITCH -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()

                    // Baseline auto
                    AutoModes.BASELINE -> getBaselineAuto()
                }

                // Cross scale auto from the sides
                "LS-RR", "LS-LR", "RS-LL", "RS-RL" -> when (crossAutoMode) {
                    // Full 2.5 cube auto
                    AutoModes.FULL -> getFullAuto(folderIn, isRightStart, scaleOwnedSide)

                    // 1 cube auto that does not interfere with other bots
                    AutoModes.SIMPLE -> getSimpleAuto(folderIn, isRightStart)

                    // Switch auto if it's on our side
                    AutoModes.SWITCH -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()

                    // Baseline auto
                    AutoModes.BASELINE -> getBaselineAuto()
                }

                // Default -- should never be returned
                else -> {
                    commandGroup { }
                }
            }
        }

        // Baseline auto
        private fun getBaselineAuto() = commandGroup {
            addSequential(StraightDriveCommand(distance = -12.0)) // Simple drive to distance
        }

        // Switch auto
        private fun getSwitchAuto(isRightStart: Boolean) = commandGroup {
            addSequential(commandGroup {
                addParallel(MotionProfileCommand("LS-LL", "Switch", robotReversed = true, pathMirrored = isRightStart)) // Go back to switch
                addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH), 1.5)
                addParallel(AutoArmCommand(ArmPosition.UP), 1.5) // Gets elevator and arm up
            })

            addSequential(TurnCommand(90.0 * if (isRightStart) 1 else -1), 1.5) // Angle correction
            addSequential(commandGroup {
                addParallel(AutoArmCommand(ArmPosition.DOWN), 1.5)
                addParallel(StraightDriveCommand(2.5), 1.0) // Drive straight with arm down
            })

            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.4, timeout = 1.0)) // Outtake cube
            addSequential(IntakeHoldCommand(), 0.001)
            addSequential(StraightDriveCommand(-2.0))
            addSequential(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Drive back and bring elevator down
        }

        // Simple auto that does not interfere with other teams' advanced auto (Worlds)
        private fun getSimpleAuto(folder: String, isRightStart: Boolean) = commandGroup {
            addSequential(commandGroup {
                addParallel(MotionProfileCommand(folder, "Simple", robotReversed = true, pathMirrored = isRightStart))
                addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH), 1.5)
                addParallel(AutoArmCommand(ArmPosition.UP), 1.5) // Go to scale
            })
            addSequential(commandGroup {
                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR)) // Elevator up
                addParallel(commandGroup {
                    addSequential(object : Command() {
                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                    })
                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 1.0, timeout = 1.0)) // Shoot cube when arm is in appropriate position
                    addSequential(IntakeHoldCommand(), 0.001)
                })
            })
        }

        // Complete 3 cube scale auto
        private fun getFullAuto(folderIn: String, isRightStart: Boolean, scaleOwnedSide: MatchData.OwnedSide) = commandGroup {

            val timeToGoUp = if (folderIn.first() == folderIn.last()) 2.50 else 1.50
            val firstCube = object : MotionProfileCommand(folderIn, "Drop First Cube", robotReversed = true, pathMirrored = isRightStart) {
                override fun isFinished(): Boolean {
                    return super.isFinished() || (ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks && !IntakeSubsystem.isCubeIn && ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100)
                }
            }

            // 1st Cube in Scale
            addSequential(commandGroup {
                addParallel(firstCube) // Go to scale
                addParallel(commandGroup { // Command that lets elevator go up to switch height
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
                        addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR)) // Elevator to scale height 
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(TimedCommand(0.1))
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = if (folderIn.first() == folderIn.last()) 0.5 else 0.65, timeout = 0.50)) // Shoot cube
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                })
            })

            // Pickup 2nd Cube
            addSequential(commandGroup {
                addSequential(commandGroup {
                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Elevator down
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 10.0)) // Intake on
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

                        var startTime: Long = 0L

                        override fun initialize() {
                            super.initialize()
                            startTime = System.currentTimeMillis()
                        }

                        override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                    }) // Path to second cube
                })
                addSequential(IntakeHoldCommand(), 0.001)

            })

            // 2nd Cube in Scale
            addSequential(commandGroup {
                val dropSecondCubePath = object : MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {
                    override fun isFinished(): Boolean {
                        return super.isFinished() || (ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks && !IntakeSubsystem.isCubeIn && ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100)
                    }
                }
                addParallel(dropSecondCubePath) // Path back to scale
                addParallel(commandGroup {
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(TimedCommand((dropSecondCubePath.pathDuration - 3.0).coerceAtLeast(0.001)))
                            addSequential(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0) // Elevator up 3 seconds before path ends
                        })
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.40, timeout = 0.50)) // Shoot cube when arm at appropriate position
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                })
            })

            // Pickup 3rd Cube
            addSequential(commandGroup {
                addSequential(commandGroup {
                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Elevator down
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

                        var startTime: Long = 0L

                        override fun initialize() {
                            super.initialize()
                            startTime = System.currentTimeMillis()
                        }

                        override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                    }) // Path to third cube
                })
                addSequential(IntakeHoldCommand(), 0.001)
            })

            // 3rd Cube in Scale
            addSequential(commandGroup {
                val dropThirdCubePath = MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT)
                addParallel(dropThirdCubePath) // Go to scale
                addParallel(commandGroup {
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(TimedCommand((dropThirdCubePath.pathDuration - 3.0).coerceAtLeast(0.001)))
                            addSequential(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0) // Elevator up 3 seconds before path ends
                        })
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.40, timeout = 0.50)) // Shoot cube when arm at appropriate position
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                })
            })

            addSequential(ElevatorPresetCommand(ElevatorPreset.INTAKE))

        }
    }
}

// Starting Position of the Robot
enum class StartingPositions {
    LEFT, CENTER, RIGHT;
}

// Auto mode to run
enum class AutoModes(val numCubes: String) {
    FULL("2.5 / 3"), SIMPLE("1"), SWITCH("0 / 1"), BASELINE("0");
}
