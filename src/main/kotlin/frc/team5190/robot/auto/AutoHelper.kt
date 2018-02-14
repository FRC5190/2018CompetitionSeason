/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.elevator.AutoElevatorCommand
import frc.team5190.robot.elevator.ElevatorPosition
import frc.team5190.robot.intake.*
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, autoMode: AutoMode): CommandGroup {

            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL", "RS-RR" -> {
                    val scale1Id = frc.team5190.robot.pathfinder.Pathfinder.requestPath("LS_LL", "Scale 1")
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH -> return commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(scale1Id, true, folder == "RS-RR"))
                                this.addParallel(commandGroup {
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                    }, 0.1)
                                    this.addSequential(TimedCommand(2.25))
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                        this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                    })
                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.1, outSpeed = 0.5))
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                this.addParallel(frc.team5190.robot.util.commandGroup {
                                    this.addSequential(TurnCommand(if (folder == "LS-LL") -15.0 else 15.0))
                                    this.addSequential(commandGroup {
                                        this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                                        this.addParallel(MotionMagicCommand(4.0))
                                    })
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                this.addParallel(MotionMagicCommand(1.3), 1.0)
                            })

                            this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                            this.addSequential(IntakeHoldCommand(), 0.001)

                            /* 3rd Cube
                            this.addSequential(MotionMagicCommand(-1.0))
                            this.addSequential(commandGroup {
                                this.addParallel(TurnCommand(124.0))
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                            })
                            this.addSequential(IntakeCommand(IntakeDirection.IN))
                            this.addSequential(MotionMagicCommand(2.5))
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE_SECOND, isReversed = true))
                                this.addParallel(commandGroup {
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE_UP))
                                        this.addParallel(AutoArmCommand(ArmPosition.UP))
                                    })
                                    this.addSequential(AutoArmCommand(ArmPosition.BEHIND))
                                })
                            })
                            this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(AutoArmCommand(ArmPosition.UP))
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                            })
                            */
                        }
                        AutoMode.TWO_CUBE_SCALE -> return commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(scale1Id, true, folder == "RS-RR"))
                                this.addParallel(commandGroup {
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                    }, 0.1)
                                    this.addSequential(TimedCommand(2.25))
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                        this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                    })
                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.1, outSpeed = 0.5))
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                this.addParallel(commandGroup {
                                    this.addSequential(TurnCommand(if (folder == "LS-LL") -15.0 else 15.0))
                                    this.addSequential(commandGroup {
                                        this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                                        this.addParallel(MotionMagicCommand(4.0))
                                    })
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                this.addParallel(MotionMagicCommand(-4.0), 1.0)
                            })

                            this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.1, outSpeed = 0.5))
                            this.addSequential(IntakeHoldCommand(), 0.001)
                        }

                    /* 3rd Cube
                    this.addSequential(MotionMagicCommand(-1.0))
                        this
                        .addSequential(commandGroup {
                        this.addParallel(TurnCommand(124.0))
                        this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                        this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                    })
                    this.addSequential(IntakeCommand(IntakeDirection.IN))
                        this
                        .addSequential(MotionMagicCommand(2.5))
                    this.addSequential(IntakeHoldCommand(), 0.001)
                        this
                        .addSequential(commandGroup {
                        this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE_SECOND, isReversed = true))
                        this.addParallel(commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE_UP))
                                this.addParallel(AutoArmCommand(ArmPosition.UP))
                            })
                            this.addSequential(AutoArmCommand(ArmPosition.BEHIND))
                        })
                    })
                    this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this
                        .addSequential(IntakeHoldCommand(), 0.001)
                    this.addSequential(AutoArmCommand(ArmPosition.UP))
                        this
                        .addSequential(commandGroup {
                        this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                        this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                    })
                    */
                    }
                }
                "LS-LR" -> {
                    TODO("Get Left Switch, Pick up cube and go to other side of scale. This is the only scenario.")
                }
                "LS-RL", "RS-LR" -> {
                    val scale1Id = frc.team5190.robot.pathfinder.Pathfinder.requestPath("LS-RL", "Scale 1")
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH -> {
                            TODO()
                        }
                        AutoMode.TWO_CUBE_SCALE -> return commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(scale1Id, true, folder == "RS-LR"))
                                this.addParallel(commandGroup {
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                    }, 0.1)
                                    this.addSequential(TimedCommand(2.25))
                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                        this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                    })
                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.1, outSpeed = 0.5))
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                this.addParallel(commandGroup {
                                    this.addSequential(TurnCommand(if (folder == "LS-RL") -15.0 else 15.0))
                                    this.addSequential(commandGroup {
                                        this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                                        this.addParallel(MotionMagicCommand(4.0))
                                    })
                                })
                            })
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(commandGroup {
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                this.addParallel(MotionMagicCommand(-4.0), 1.0)
                            })

                            this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.1, outSpeed = 0.5))
                            this.addSequential(IntakeHoldCommand(), 0.001)
                        }

                    // No 3 Cube Autonomous
                    }
                }
                "LS-RR" -> {
                    TODO("Drop in right scale, pick up cube and drop in right switch. This is the only scenario.")
                }
                "CS-L" -> {
                    TODO("Drop in left switch, pick up cube from pyramid, push into exchange.")
                }
                "CS-R" -> {
                    val switchId = frc.team5190.robot.pathfinder.Pathfinder.requestPath("CS-R", "Switch")
                    val centerId = frc.team5190.robot.pathfinder.Pathfinder.requestPath("CS-R", "Center")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))

                        this.addSequential(commandGroup {
                            this.addParallel(IntakeHoldCommand(), 0.001)
                            this.addParallel(MotionProfileCommand(centerId, true))
                        })

                        this.addSequential(commandGroup {
                            this.addParallel(TurnCommand(0.0, true, 10.0))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                            this.addParallel(MotionMagicCommand(4.0))
                        })

                        this.addSequential(commandGroup {
                            this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 3.0))
                        })

                        this.addSequential(IntakeHoldCommand(), 0.001)

                        this.addSequential(MotionMagicCommand(-1.5))
                    }
                }

                "RS-LL" -> {
                    TODO("Drop in left scale, pick up cube and drop in left switch. This is the only scenario.")
                }
                "RS-LR" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH -> TODO()
                        AutoMode.TWO_CUBE_SCALE -> TODO()
                    }
                }
                "RS-RL" -> {
                    TODO("Drop in right switch, Pick up cube and go to other side of scale. This is the only scenario.")
                }
                "RS-RR" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH -> TODO()
                        AutoMode.TWO_CUBE_SCALE -> TODO()
                    }
                }
                else -> TODO("Does not exist.")
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


enum class AutoMode {
    TWO_CUBE_SCALE, TWO_CUBE_BOTH
}
