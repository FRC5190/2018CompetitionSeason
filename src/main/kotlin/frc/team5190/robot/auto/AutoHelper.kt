/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.*
import frc.team5190.robot.arm.*
import frc.team5190.robot.elevator.*
import frc.team5190.robot.intake.*
import frc.team5190.robot.pathreader.Pathreader
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
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, lslr: String): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL", "RS-RR" -> {
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        addSequential(goToAndDropCubeOnScale(scale1Id, folder == "RS-RR"))
                        addSequential(pickupCube(folder == "LS-LL"))
                        addSequential(dropCubeOnSwitch())

                        addSequential(commandGroup {
                            addParallel(MotionMagicCommand(-2.0), 1.2)
                            addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            addParallel(AutoArmCommand(ArmPosition.DOWN))
                        })
                        addSequential(commandGroup {
                            addSequential(TurnCommand(if (folder == "LS-LL") -50.0 else 37.5, visionCheck = true, tolerance = 10.0))
                            addSequential(commandGroup {
                                addParallel(MotionMagicCommand(4.0, cruiseVel = 5.0), 1.2)
                                addParallel(commandGroup {
                                    addSequential(IntakeCommand(IntakeDirection.IN, timeout = 2.25, inSpeed = 0.75))
                                    addSequential(IntakeHoldCommand(), 0.001)

                                })
                            })
                        })


                        addSequential(TurnCommand(3.0), 0.75)
                        addSequential(dropCubeOnSwitch())
                    }
                }

                "LS-RL", "RS-LR" -> {
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        addSequential(goToAndDropCubeOnScale(scale1Id, folder == "RS-LR"))
                        addSequential(pickupCube(folder == "LS-RL"))
                        addSequential(switchToScale(folder == "RS-LR"))
                    }
                }

                "LS-LR", "RS-RL" -> {
                    when (lslr) {
                        "2 Switch" -> {
                            val switchId = Pathreader.requestPath("LS-LR", "Switch")
                            return commandGroup {
                                addSequential(goToSwitch(switchId, folder == "RS-RL"))
                                addSequential(TurnCommand(-10.0, visionCheck = true, tolerance = 12.0))

                                addSequential(MotionMagicCommand(6.0), 1.5)
                                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.5, outSpeed = 0.4))
                                addSequential(IntakeHoldCommand(), 0.001)

                                addSequential(commandGroup {
                                    addParallel(MotionMagicCommand(-1.5), 1.0)
                                    addParallel(commandGroup {
                                        addSequential(TimedCommand(0.5))
                                        addSequential(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                    })
                                    addParallel(AutoArmCommand(ArmPosition.DOWN))
                                })

                                addSequential(pickupCube(folder == "LS-LR", 1.5))
                                addSequential(dropCubeOnSwitch())
                            }
                        }
                        "2 Scale" -> {
                            val scaleId = Pathreader.requestPath("LS-RR", "Scale")
                            return commandGroup {
                                addSequential(goToAndDropCubeOnScale(scaleId, folder == "RS-RL"))
                                addSequential(pickupCube(folder == "RS-RL"))
                                addSequential(switchToScale(folder == "RS-RL"))
                            }
                        }

                        else -> throw IllegalArgumentException("Scenario does not exist.")
                    }
                }

                "LS-RR", "RS-LL" -> {
                    val scaleId = Pathreader.requestPath("LS-RR", "Scale")
                    return commandGroup {
                        addSequential(goToAndDropCubeOnScale(scaleId, folder == "RS-LL"))
                        addSequential(pickupCube(folder == "RS-LL"))
                        addSequential(dropCubeOnSwitch())
                    }
                }

                "CS-L" -> {
                    val switchId = Pathreader.requestPath("CS-L", "Switch")
                    val centerId = Pathreader.requestPath("CS-L", "Center")
                    val switch2Id = Pathreader.requestPath("CS-L", "Switch 2")
                    return commandGroup {
                        addSequential(dropCubeFromCenter(switchId))
                        addSequential(getBackToCenter(centerId))
                        addSequential(pickupCubeFromCenter())
                        addSequential(dropCubeFromCenter(switch2Id))
                        addSequential((MotionMagicCommand(-2.00)))
                    }
                }

                "CS-R" -> {
                    val switchId = Pathreader.requestPath("CS-R", "Switch")
                    val centerId = Pathreader.requestPath("CS-R", "Center")
                    val switch2Id = Pathreader.requestPath("CS-R", "Switch 2")
                    return commandGroup {
                        addSequential(dropCubeFromCenter(switchId))
                        addSequential(getBackToCenter(centerId))
                        addSequential(pickupCubeFromCenter())
                        addSequential(dropCubeFromCenter(switch2Id))
                        addSequential((MotionMagicCommand(-2.00)))
                    }
                }

                else -> throw IllegalArgumentException("Scenario does not exist.")
            }
        }

        /**
         * Goes from switch to scale.
         */
        private fun switchToScale(isLeft: Boolean): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                    addParallel(AutoArmCommand(ArmPosition.BEHIND))
                    addParallel(commandGroup {
                        addSequential(TimedCommand(0.01))
                        addSequential(MotionMagicCommand(-4.5))
                        addSequential(TurnCommand(if (isLeft) 12.5 else -12.5))
                    })
                })

                addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 1.0, timeout = 1.0))
                addSequential(AutoArmCommand(ArmPosition.MIDDLE))
            }

        }

        /**
         * Picks up a cube using Vision
         * @param leftTurn Whether the turn is to the left
         */
        private fun pickupCube(leftTurn: Boolean, mmDistanceFeet: Double = 6.0): CommandGroup {
            return commandGroup {
                addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                addParallel(AutoArmCommand(ArmPosition.DOWN))
                addParallel(commandGroup {
                    addSequential(TurnCommand(if (leftTurn) -10.0 else 5.0, visionCheck = true, tolerance = 12.0))
                    addSequential(commandGroup {
                        addParallel(MotionMagicCommand(mmDistanceFeet, cruiseVel = 5.0), 1.2)
                        addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.25, inSpeed = 1.0))
                    })
                    addSequential(IntakeHoldCommand(), 0.001)
                })
            }
        }

        /**
         * Drops the cube on the scale
         * @param scaleId ID of the scale MP
         * @param isMirrored Whether the MP is mirrored
         * @param isOpposite whether the scale is on the opposite side of the starting position
         */
        private fun goToAndDropCubeOnScale(scaleId: Int, isMirrored: Boolean): CommandGroup {

            val mpCommand = MotionProfileCommand(scaleId, true, isMirrored)
            val mpDuration = mpCommand.getMPTime()

            return commandGroup {
                addSequential(commandGroup {
                    addParallel(mpCommand)
                    addParallel(commandGroup {
                        addSequential(TimedCommand(0.5))
                        addSequential(commandGroup {
                            addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            addParallel(AutoArmCommand(ArmPosition.UP))
                        })
                        addSequential(TimedCommand(mpDuration - 3.5))
                        addSequential(commandGroup {
                            addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                            addParallel(AutoArmCommand(ArmPosition.BEHIND))
                            addParallel(commandGroup {
                                addSequential(TimedCommand(1.15))
                                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.325, outSpeed = 0.45))
                                addSequential(IntakeHoldCommand(), 0.001)
                            })
                        })
                    })
                })
            }
        }

        /**
         * Goes to the switch
         * @param scaleID ID of the scale MP
         * @param isMirrored Whether the MP is mirrored
         */
        private fun goToSwitch(scaleId: Int, isMirrored: Boolean): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(MotionProfileCommand(scaleId, true, isMirrored))
                    addParallel(commandGroup {
                        addSequential(TimedCommand(0.5))
                        addSequential(commandGroup {
                            addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            addParallel(AutoArmCommand(ArmPosition.UP))
                        }, 1.0)
                        addSequential(TimedCommand(3.0))
                        addSequential(commandGroup {
                            addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })
                    })
                })
            }
        }

        /**
         * Drops the cube on the switch
         */
        private fun dropCubeOnSwitch(mmDistanceFeet: Double = 1.1, mTimeout: Double = 1.0): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                    addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                    addParallel(commandGroup {
                        addSequential(MotionMagicCommand(-0.2))
                        addSequential(object : Command() {
                            override fun isFinished() =
                                    ElevatorSubsystem.currentPosition > ElevatorPosition.SWITCH.ticks - 1440 && ArmSubsystem.currentPosition > ArmPosition.MIDDLE.ticks - 400
                        })
                        addSequential(commandGroup {
                            addParallel(MotionMagicCommand(mmDistanceFeet), mTimeout)
                            addParallel(commandGroup {
                                addSequential(TimedCommand(0.5))
                                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.37))
                            })
                        })
                    })
                })
                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                addSequential(IntakeHoldCommand(), 0.001)
            }
        }

        /**
         * Drops the cube from the center
         * @param switchId ID of the switch MP
         */
        private fun dropCubeFromCenter(switchId: Int): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(MotionProfileCommand(switchId))
                    addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                    addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                })

                addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                addSequential(IntakeHoldCommand(), 0.001)
            }
        }

        /**
         * Goes from the switch to the center position
         * @param centerId ID of the center MP
         */
        private fun getBackToCenter(centerId: Int): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(MotionProfileCommand(centerId, true))
                    addParallel(commandGroup {
                        addSequential(TimedCommand(0.5))
                        addSequential(commandGroup {
                            addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            addParallel(AutoArmCommand(ArmPosition.DOWN))
                        })
                    })
                })
                addSequential(commandGroup {
                    addParallel(TurnCommand(0.0, false, 0.0))

                })
            }
        }

        /**
         * Picks up a cube from the center pyramid
         */
        private fun pickupCubeFromCenter(): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    addParallel(MotionMagicCommand(4.00))
                    addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                })
                addSequential(IntakeHoldCommand(), 0.001)
                addSequential(MotionMagicCommand(-4.25, cruiseVel = 5.0, accel = 4.0), 1.2)
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
