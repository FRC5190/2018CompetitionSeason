/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.*
import frc.team5190.robot.arm.*
import frc.team5190.robot.drive.StraightDriveCommand
import frc.team5190.robot.drive.TurnCommand
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
                "CS-L", "CS-R" -> commandGroup {
                    val firstSwitch = MotionProfileCommand(folder, "Drop First Cube", robotReversed = true)

                    // Drop First Cube
                    addSequential(commandGroup {
                        addParallel(firstSwitch)
                        addParallel(commandGroup {
                            addSequential(commandGroup {
                                addParallel(AutoElevatorCommand(ElevatorPosition.FIRST_STAGE), 1.0)
                                addParallel(AutoArmCommand(ArmPosition.UP), 0.5)
                            })
                            addSequential(AutoArmCommand(ArmPosition.BEHIND))
                        })
                        addParallel(commandGroup {
                            addSequential(TimedCommand(firstSwitch.pathDuration - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.3, timeout = 0.5))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })

                    /*
                    // Pickup Second Cube
                    addSequential(commandGroup {
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                        addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 10.0))
                        addParallel(object : MotionProfileCommand("CS-L", "Pickup Second Cube", pathReversed = true, pathMirrored = folder.first() == 'R') {
                            var startTime: Long = 0L

                            override fun initialize() {
                                super.initialize()
                                startTime = System.currentTimeMillis()
                            }

                            override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                        })
                    })

                    // Drop Second Cube
                    addSequential(commandGroup {

                        val dropSecondCubePath = MotionProfileCommand("CS-L", "Pickup Second Cube", robotReversed = true, pathMirrored = folder.first() == 'R')

                        addParallel(dropSecondCubePath)
                        addParallel(commandGroup {
                            addSequential(AutoElevatorCommand(ElevatorPosition.FIRST_STAGE))
                            addSequential(AutoArmCommand(ArmPosition.BEHIND))
                        })
                        addParallel(commandGroup {
                            addSequential(TimedCommand(dropSecondCubePath.pathDuration - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.5, timeout = 0.5))
                        })
                    })

                    // Pickup Third Cube
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(AutoArmCommand(ArmPosition.DOWN))
                            addSequential(AutoElevatorCommand(4000.0))
                        })
                        addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 10.0))
                        addParallel(object : MotionProfileCommand("CS-L", "Pickup Third Cube", pathReversed = true, pathMirrored = folder.first() == 'R') {
                            var startTime: Long = 0L

                            override fun initialize() {
                                super.initialize()
                                startTime = System.currentTimeMillis()
                            }

                            override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                        })
                    })
                    */
                }

                "LS-LL", "LS-RL", "RS-RR", "RS-LR" -> when (sameSideAutoMode) {
                    AutoModes.FULL -> getFullAuto(folderIn, isRightStart, scaleOwnedSide)
                    AutoModes.SIMPLE -> getSimpleAuto(folderIn, isRightStart)
                    AutoModes.SWITCH -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()
                    AutoModes.BASELINE -> getBaselineAuto()
                }

                "LS-RR", "LS-LR", "RS-LL", "RS-RL" -> when (crossAutoMode) {
                    AutoModes.FULL -> getFullAuto(folderIn, isRightStart, scaleOwnedSide)
                    AutoModes.SIMPLE -> getSimpleAuto(folderIn, isRightStart)
                    AutoModes.SWITCH -> if (switchOwnedSide.name.first().toUpperCase() == folder.first()) getSwitchAuto(isRightStart) else getBaselineAuto()
                    AutoModes.BASELINE -> getBaselineAuto()
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

        private fun getFullAuto(folderIn: String, isRightStart: Boolean, scaleOwnedSide: MatchData.OwnedSide) = commandGroup {

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
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.50, timeout = 0.50))
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
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 10.0))
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

                        var startTime: Long = 0L

                        override fun initialize() {
                            super.initialize()
                            startTime = System.currentTimeMillis()
                        }

                        override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                    })
                })
                addSequential(IntakeHoldCommand(), 0.001)

            })

            /*
             Drop 2nd Cube in Scale
              */
            addSequential(commandGroup {
                val dropSecondCubePath = object : MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {
                    override fun isFinished(): Boolean {
                         return super.isFinished() || (ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks && !IntakeSubsystem.isCubeIn && ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100)
                    }
                }
                addParallel(dropSecondCubePath)
                addParallel(commandGroup {
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(TimedCommand((dropSecondCubePath.pathDuration - 3.0).coerceAtLeast(0.001)))
                            addSequential(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                        })
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
        Pickup 3rd Cube
         */
            addSequential(commandGroup {
                addSequential(commandGroup {
                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                    addParallel(object : MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

                        var startTime: Long = 0L

                        override fun initialize() {
                            super.initialize()
                            startTime = System.currentTimeMillis()
                        }

                        override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                    })
                })
                addSequential(IntakeHoldCommand(), 0.001)
            })

            /*
             Drop 3rd Cube in Scale
              */
            addSequential(commandGroup {
                val dropThirdCubePath = MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT)
                addParallel(dropThirdCubePath)
                addParallel(commandGroup {
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(TimedCommand((dropThirdCubePath.pathDuration - 3.0).coerceAtLeast(0.001)))
                            addSequential(ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR), 3.0)
                        })
                        addParallel(commandGroup {
                            addSequential(object : Command() {
                                override fun isFinished() = ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100
                            })
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 0.35, timeout = 0.50))
                            addSequential(IntakeHoldCommand(), 0.001)
                        })
                    })
                })
            })

            addSequential(ElevatorPresetCommand(ElevatorPreset.INTAKE))

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
