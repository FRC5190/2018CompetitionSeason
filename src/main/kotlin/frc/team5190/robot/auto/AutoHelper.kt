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
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, sameSideAutoMode: AutoModes, crossAutoMode: AutoModes): CommandGroup {

            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            val isRightStart = folder.first() == 'R'
            val folderIn = if (folder.first() == folder.last()) "LS-LL" else "LS-RR"

            return when (folder) {
                "CS-L, CS-R" -> commandGroup {
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

                "LS-LL", "LS-RL", "RS-RR", "RS-LR" -> when (sameSideAutoMode) {
                    AutoModes.FULL      -> getFullAuto(folderIn, isRightStart)
                    AutoModes.SIMPLE    -> getSimpleAuto(folderIn, isRightStart)
                    AutoModes.SWITCH    -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()
                    AutoModes.BASELINE  -> getBaselineAuto()
                }

                "LS-RR", "LS-LR", "RS-LL", "RS-RL" -> when (crossAutoMode) {
                    AutoModes.FULL      -> getFullAuto(folderIn, isRightStart)
                    AutoModes.SIMPLE    -> getSimpleAuto(folderIn, isRightStart)
                    AutoModes.SWITCH    -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()
                    AutoModes.BASELINE  -> getBaselineAuto()
                }

                else -> {
                    commandGroup { }
                }
            }
        }


        private fun getBaselineAuto() = commandGroup {
            addSequential(StraightDriveCommand(distance = -12.0))
        }

        private fun getSwitchAuto(isRightStart: Boolean) = commandGroup {
            addSequential(commandGroup {
                addParallel(MotionProfileCommand("LS-LL", "Switch", robotReversed = true, pathMirrored = isRightStart))
                addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                addParallel(AutoArmCommand(ArmPosition.UP))
            })

            addSequential(TurnCommand(90.0 * if (isRightStart) -1 else 1))
            addSequential(commandGroup {
                addParallel(AutoArmCommand(ArmPosition.DOWN))
                addParallel(StraightDriveCommand(2.5), 1.0)
            })

            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.4, timeout = 0.2))
            addSequential(IntakeHoldCommand(), 0.001)
            addSequential(StraightDriveCommand(-2.0))
            addSequential(ElevatorPresetCommand(ElevatorPreset.INTAKE))
        }

        private fun getSimpleAuto(folder: String, isRightStart: Boolean) = commandGroup {
            addSequential(commandGroup {
                addParallel(MotionProfileCommand(folder, "Non Interfering", robotReversed = true, pathMirrored = isRightStart))
                addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                addParallel(AutoArmCommand(ArmPosition.UP))
            })
            addSequential(commandGroup {
                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR))
                addParallel(commandGroup {
                    addSequential(object : Command() {
                        override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                    })
                    addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.75, timeout = 0.5))
                    addSequential(IntakeHoldCommand(), 0.001)
                })
            })
        }

        private fun getFullAuto(folderIn: String, isRightStart: Boolean) = commandGroup {

            val timeToGoUp = if (folderIn.first() == folderIn.last()) 2.50 else 1.50
            val firstCube = MotionProfileCommand(folderIn, "Drop First Cube", robotReversed = true, pathMirrored = isRightStart)

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
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = isRightStart) {
                        override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                    })
                })
                addSequential(IntakeHoldCommand(), 0.001)

            })

            /*
             Drop 2nd Cube in Scale
              */
            addSequential(commandGroup {
                val dropSecondCubePath = MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = isRightStart)
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

            /*
        Pickup 3rd Cube
         */
            addSequential(commandGroup {
                addSequential(commandGroup {
                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = isRightStart) {
                        override fun isFinished() = super.isFinished() || IntakeSubsystem.isCubeIn
                    })
                })
                addSequential(IntakeHoldCommand(), 0.001)
            })

            /*
             Go to Scale with 3rd Cube
              */
            addSequential(commandGroup {
                val dropThirdCubePath = MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = isRightStart)
                addParallel(dropThirdCubePath)
                addParallel(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
            })

        }
    }
}

/**
 * Stores starting position of the robot.
 */
enum class StartingPositions {
    LEFT, CENTER, RIGHT;
}

enum class AutoModes(val numCubes: String) {
    FULL("2.5 / 3"), SIMPLE("1"), SWITCH("0 / 1"), BASELINE("0");
}
