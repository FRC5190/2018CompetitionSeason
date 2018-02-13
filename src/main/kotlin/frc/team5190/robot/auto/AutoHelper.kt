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
import frc.team5190.robot.pathfinder.Pathfinder
import frc.team5190.robot.util.commandGroup
import frc.team5190.robot.vision.FindCubeCommand
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
                    val scaleId = Pathfinder.requestPath("LS-LL", "Scale")
                    val scaleSecondId = Pathfinder.requestPath("LS-LL", "Scale Second")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(scaleId, true, folder == "RS-RR"))
                            // Move elevator up during the motion profile
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
                                this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.3, outSpeed = 0.5))
                            })
                        })
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        // 2nd cube
                        //this.addSequential(AutoArmCommand(ArmPosition.MIDDLE))
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                            this.addParallel(frc.team5190.robot.util.commandGroup {
                                this.addSequential(TurnCommand(-15.0))
                                this.addSequential(commandGroup {
                                    this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 2.0))
                                    this.addParallel(MotionMagicCommand(4.0))
                                })
                            })
                        })
                        //this.addSequential(MotionMagicCommand(3.0))
                        //this.addSequential(FindCubeCommand())
                        //this.addSequential(MotionMagicCommand((VisionSubsystem.tgtRange_in - 10).coerceAtLeast(0.0) / 12))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                            this.addParallel(MotionMagicCommand(1.3), 1.0)
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        // 3rd cube
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
                            this.addParallel(MotionProfileCommand(scaleSecondId, isReversed = true, isMirrored = (folder == "RS-RR")))
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
                    }
                    /*
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(Paths.LS_LL_SWITCH))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                    }*/
                }
                "LS-RL" -> {
                    val scaleId = Pathfinder.requestPath(folder, "Scale")
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(scaleId))
                            // Move elevator up during the motion profile
                            this.addParallel(commandGroup {
                                this.addSequential(commandGroup {
                                    this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                    this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                }, 0.1)
                                this.addSequential(TimedCommand(2.5))
                                this.addSequential(commandGroup {
                                    this.addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                                    this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                })
                            })
                        })
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                    }
                }
                "CS-R" -> {
                    val switchId = Pathfinder.requestPath(folder, "Switch")
                    val centerId = Pathfinder.requestPath(folder, "Center")
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
                            this.addParallel(FindCubeCommand())
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                        })

                        this.addSequential(commandGroup {
                            this.addParallel(MotionMagicCommand((VisionSubsystem.tgtRange_in - 10.0) / 12.0))
                            this.addParallel(IntakeCommand(IntakeDirection.IN, timeout = 3.0))
                        })

                        this.addSequential(IntakeHoldCommand(), 0.001)

                        this.addSequential(MotionMagicCommand(-1.5))
                    }
                }

                else -> TODO("Generate paths")
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
