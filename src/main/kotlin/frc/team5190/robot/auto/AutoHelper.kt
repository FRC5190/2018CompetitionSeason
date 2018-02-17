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
import frc.team5190.robot.intake.IntakeCommand
import frc.team5190.robot.intake.IntakeDirection
import frc.team5190.robot.intake.IntakeHoldCommand
import frc.team5190.robot.pathreader.Pathreader
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
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        this.addSequential(dropCubeOnScaleSame(scale1Id, folder == "RS-RR"))
                        this.addSequential(pickupCube(folder == "LS-LL"))
                        this.addSequential(dropCubeOnSwitch())
                    }
                }
                "LS-RL", "RS-LR" -> {
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        this.addSequential(dropCubeOnScaleSame(scale1Id, folder == "RS-LR"))
                        this.addSequential(pickupCube(folder == "LS-RL"))
                    }
                }
                "LS-LR", "RS-RL" -> {
                    val switchId = Pathreader.requestPath("LS-LR", "Switch")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId, true, folder == "RS-RL"))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })
                        this.addSequential(MotionMagicCommand(2.0))
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                    }
                }
                "LS-RR", "RS-LL" -> {
                    val scaleId = Pathreader.requestPath("LS-RR", "Scale")
                    return commandGroup {
                        this.addSequential(dropCubeOnScaleOpposite(scaleId, folder == "RS-LL"))
                        this.addSequential(pickupCube(folder == "RS-LL"))
                        this.addSequential(dropCubeOnSwitch())
                    }
                }
                "CS-L" -> {
                    val switchId = Pathreader.requestPath("CS-L", "Switch")
                    val centerId = Pathreader.requestPath("CS-L", "Center")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        this.addSequential(MotionProfileCommand(centerId, true))
                        this.addSequential(pickupCubeFromCenter())
                    }
                }
                "CS-R" -> {
                    val switchId = Pathreader.requestPath("CS-R", "Switch")
                    val centerId = Pathreader.requestPath("CS-R", "Center")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        this.addSequential(MotionProfileCommand(centerId, true))
                        this.addSequential(pickupCubeFromCenter())
                    }
                }
                else -> TODO("Does not exist.")
            }
        }

        private fun dropCubeOnScaleSame(scaleId: Int, isMirrored: Boolean): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    this.addParallel(MotionProfileCommand(scaleId, true, isMirrored))
                    this.addParallel(commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.UP))
                        }, 0.1)
                        this.addSequential(TimedCommand(2.25))
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                            this.addParallel(commandGroup {
                                this.addSequential(TimedCommand(0.25))
                                this.addSequential(AutoArmCommand(ArmPosition.BEHIND))
                            })
                        })
                        this.addSequential(TimedCommand(0.25))
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.4, outSpeed = 0.55))
                    })
                })
                this.addSequential(IntakeHoldCommand(), 0.001)
            }
        }

        private fun dropCubeOnScaleOpposite(scaleId: Int, isMirrored: Boolean): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    this.addParallel(MotionProfileCommand(scaleId, true, isMirrored))
                    this.addParallel(commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.UP))
                        }, 0.1)
                        this.addSequential(TimedCommand(5.25))
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                            this.addParallel(commandGroup {
                                this.addSequential(TimedCommand(0.25))
                                this.addSequential(AutoArmCommand(ArmPosition.BEHIND))
                            })
                        })
                        this.addSequential(TimedCommand(0.25))
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 1.0, outSpeed = 1.0))
                    })
                })
                this.addSequential(IntakeHoldCommand(), 0.001)
            }
        }

        private fun pickupCube(leftTurn: Boolean): CommandGroup {
            return commandGroup {
                this.addSequential(commandGroup {
                    this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                    this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                    this.addParallel(TurnCommand(if (leftTurn) -10.0 else 10.0, visionCheck = true, tolerance = 15.0))
                })
                this.addSequential(frc.team5190.robot.util.commandGroup {
                    this.addSequential(commandGroup {
                        this.addParallel(frc.team5190.robot.util.commandGroup {
                            this.addSequential(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                            this.addSequential(IntakeHoldCommand(), 0.001)
                        })
                        this.addParallel(MotionMagicCommand(4.0), 1.0)
                    })
                })
            }
        }

        private fun dropCubeOnSwitch(): CommandGroup {
            return commandGroup {
                this.addSequential(MotionMagicCommand(-0.25))
                this.addSequential(commandGroup {
                    this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                    this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                })
                this.addSequential(MotionMagicCommand(1.3), 1.0)
                this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                this.addSequential(IntakeHoldCommand(), 0.001)
            }
        }

        private fun pickupCubeFromCenter(): CommandGroup {
            return commandGroup {
                this.addSequential(commandGroup {
                    this.addParallel(TurnCommand(0.0, true, 15.0))
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
