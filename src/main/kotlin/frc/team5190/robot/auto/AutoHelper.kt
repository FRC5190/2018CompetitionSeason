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
import frc.team5190.robot.vision.VisionSubsystem
import openrio.powerup.MatchData

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide): CommandGroup {

            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL", "RS-RR" -> {
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        this.addSequential(dropCubeOnScale(scale1Id, folder == "RS-RR", false))
                        this.addSequential(pickupCube(folder == "LS-LL"))
                        this.addSequential(dropCubeOnSwitch())
                    }
                }
                "LS-RL", "RS-LR" -> {
                    val scale1Id = Pathreader.requestPath("LS-LL", "Scale")
                    return commandGroup {
                        this.addSequential(dropCubeOnScale(scale1Id, folder == "RS-LR", false))
                        this.addSequential(pickupCube(folder == "LS-RL"))
                    }
                }
                "LS-LR", "RS-RL" -> {
                    val switchId = Pathreader.requestPath("LS-LR", "Switch")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId, true, folder == "RS-RL"))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.UP))
                        })
                        this.addSequential(TurnCommand(-90.0, false))
                        this.addSequential(frc.team5190.robot.util.commandGroup {
                            this.addParallel(MotionMagicCommand(2.5), 1.0)
                            this.addParallel(AutoArmCommand(frc.team5190.robot.arm.ArmPosition.MIDDLE))
                        })
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                    }
                }
                "LS-RR", "RS-LL" -> {
                    val scaleId = Pathreader.requestPath("LS-RR", "Scale")
                    return commandGroup {
//                        this.addSequential(dropCubeOnScale(scaleId, folder == "RS-LL", true))
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

                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(switchId))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)

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

                        this.addSequential(frc.team5190.robot.util.commandGroup {
                            this.addParallel(MotionProfileCommand(switchId))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                    }
                }
                else -> TODO("Does not exist.")
            }
        }

        private fun pickupCube(leftTurn: Boolean): CommandGroup {
            return commandGroup {
                this.addParallel(commandGroup {
                    this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                    this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                })
                this.addParallel(commandGroup {
                    this.addSequential(TurnCommand(if (leftTurn) -10.0 else 10.0, visionCheck = true, tolerance = 15.0))
                    this.addSequential(commandGroup {
                        this.addParallel(MotionMagicCommand(5.0, cruiseVel = 5.0))
                        this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.75, inSpeed = 1.0))
                    })
                    this.addSequential(IntakeHoldCommand(), 0.001)
                })
            }
        }

        private fun dropCubeOnScale(scaleId: Int, isMirrored: Boolean, isOpposite: Boolean): CommandGroup {
            return commandGroup {
                addSequential(commandGroup {
                    this.addParallel(MotionProfileCommand(scaleId, true, isMirrored))
                    this.addParallel(commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.UP))
                        }, 0.1)
                        this.addSequential(TimedCommand(if (isOpposite) 5.0 else 2.25))
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
                    this.addParallel(TurnCommand(0.0, false, 0.0))
                    this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                    this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                })

                this.addSequential(frc.team5190.robot.util.commandGroup {
                    this.addParallel(MotionMagicCommand(4.00))
                    this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                })

                this.addSequential(IntakeHoldCommand(), 0.001)
                this.addSequential(frc.team5190.robot.util.commandGroup {
                    this.addParallel(MotionMagicCommand(-5.0))
                    this.addParallel(AutoElevatorCommand(frc.team5190.robot.elevator.ElevatorPosition.SWITCH))
                    this.addParallel(AutoArmCommand(frc.team5190.robot.arm.ArmPosition.MIDDLE))
                })
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
