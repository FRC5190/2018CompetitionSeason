package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.elevator.*
import frc.team5190.robot.intake.IntakeCommand
import frc.team5190.robot.intake.IntakeDirection
import frc.team5190.robot.intake.IntakeHoldCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

@Suppress("unused")
class AutoHelper2 {
    companion object {
        fun getAuto(startingPositions: StartingPositions,
                    switchOwnedSide: MatchData.OwnedSide,
                    scaleOwnedSide: MatchData.OwnedSide,
                    sameSideAutoMode: AutoModes,
                    crossAutoMode: AutoModes): CommandGroup {

            return when (startingPositions) {
                StartingPositions.CENTER -> commandGroup {
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
                        addParallel(MotionProfileCommand2(FastTrajectories.switchToCenter, pathMirrored = switchOwnedSide == MatchData.OwnedSide.RIGHT)) // Go back to center
                        addParallel(commandGroup {
                            addSequential(TimedCommand(0.5)) // Wait 0.5 seconds
                            addSequential(commandGroup {
                                addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Get elevator to intaking position
                            })
                        })
                    })
                    addSequential(commandGroup {
                        addParallel(IntakeCommand(IntakeDirection.IN, timeout = 3.0)) // pickup pyramid cube
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

                StartingPositions.RIGHT, StartingPositions.LEFT -> {
                    if (startingPositions.name.first().toUpperCase() == scaleOwnedSide.name.first().toUpperCase()) {
                        when (sameSideAutoMode) {
                            AutoModes.BASELINE -> commandGroup {
                                addSequential(MotionProfileCommand2(FastTrajectories.leftStartToNearScale, startingPositions == StartingPositions.RIGHT))
                            }

                            AutoModes.FULL, AutoModes.SIMPLE, AutoModes.SWITCH -> getFullAuto(startingPositions, scaleOwnedSide)
                        }
                    } else {
                        when (crossAutoMode) {
                            AutoModes.BASELINE -> commandGroup {
                                addSequential(MotionProfileCommand2(FastTrajectories.leftStartToFarScale, startingPositions == StartingPositions.RIGHT))
                            }

                            AutoModes.FULL, AutoModes.SIMPLE, AutoModes.SWITCH -> getFullAuto(startingPositions, scaleOwnedSide)
                        }
                    }
                }
            }
        }

        // Complete 3 cube scale auto
        private fun getFullAuto(startingPositions: StartingPositions, scaleOwnedSide: MatchData.OwnedSide) = commandGroup {
            val nearScale = startingPositions.name.first().toUpperCase() == scaleOwnedSide.name.first().toUpperCase()

            val timeToGoUp = if (nearScale) 1.75 else 1.50
            val firstCube = object : MotionProfileCommand2(if (nearScale) FastTrajectories.leftStartToNearScale else FastTrajectories.leftStartToFarScale,
                    pathMirrored = startingPositions == StartingPositions.RIGHT) {

                override fun isFinished(): Boolean {
                    return super.isFinished() ||
                            (ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks &&
                                    !IntakeSubsystem.isCubeIn && ArmSubsystem.currentPosition > ArmPosition.BEHIND.ticks - 100)
                }
            }

            // 1st Cube in Scale
            addSequential(commandGroup {
                addParallel(firstCube) // Go to scale
                addParallel(commandGroup {
                    // Command that lets elevator go up to switch height
                    addSequential(TimedCommand(0.2))
                    addSequential(object : CommandGroup() {
                        var startTime: Long = 0

                        init {
                            addParallel(AutoElevatorCommand(ElevatorPosition.FIRST_STAGE))
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
                            addSequential(IntakeCommand(IntakeDirection.OUT, speed = if (nearScale) 0.35 else 0.65, timeout = 0.50)) // Shoot cube
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
                    addParallel(commandGroup {
                        addSequential(TimedCommand(0.3))
                        addSequential(object : MotionProfileCommand2(FastTrajectories.scaleToCube1, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

                            var startTime: Long = 0L

                            override fun initialize() {
                                super.initialize()
                                startTime = System.currentTimeMillis()
                            }

                            override fun isFinished() = super.isFinished() || ((System.currentTimeMillis() - startTime > 750) && IntakeSubsystem.isCubeIn)
                        }) //
                    })

                })
                addSequential(IntakeHoldCommand(), 0.001)

            })

            // 2nd Cube in Scale
            addSequential(commandGroup {
                val dropSecondCubePath = object : MotionProfileCommand2(FastTrajectories.cube1ToScale, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {
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
                    addParallel(object : MotionProfileCommand2(FastTrajectories.scaleToCube2, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

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
                val dropThirdCubePath = MotionProfileCommand2(FastTrajectories.cube2ToScale, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT)
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

            // Pickup 4th Cube
            addSequential(commandGroup {
                addSequential(commandGroup {
                    addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE)) // Elevator down
                    addParallel(IntakeCommand(IntakeDirection.IN, speed = 1.0, timeout = 5.0))
                    addParallel(object : MotionProfileCommand2(FastTrajectories.scaleToCube3, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT) {

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

            // 4th Cube in Scale
            addSequential(commandGroup {
                val dropFourthCubePath = MotionProfileCommand2(FastTrajectories.cube3ToScale, pathMirrored = scaleOwnedSide == MatchData.OwnedSide.RIGHT)
                addParallel(dropFourthCubePath) // Go to scale
                addParallel(commandGroup {
                    addSequential(commandGroup {
                        addParallel(commandGroup {
                            addSequential(TimedCommand((dropFourthCubePath.pathDuration - 3.0).coerceAtLeast(0.001)))
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


        }
    }
}