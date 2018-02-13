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
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.Maths
import frc.team5190.robot.util.commandGroup
import frc.team5190.robot.vision.FindCubeCommand
import frc.team5190.robot.vision.VisionSubsystem
import openrio.powerup.MatchData
import java.io.InputStreamReader

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, autoMode: AutoMode): CommandGroup {

            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH  -> return commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE1, true))
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
                                    this.addSequential(TurnCommand(-15.0))
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
                                this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE1, true))
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
                                    this.addSequential(TurnCommand(-15.0))
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
                "LS-RL" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH  -> {TODO()}
                        AutoMode.TWO_CUBE_SCALE -> return commandGroup {
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.LS_RL_SCALE1, true))
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
                                    this.addSequential(TurnCommand(-15.0))
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
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(Paths.CS_R_SWITCH))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                        })

                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.2, outSpeed = 0.5))

                        this.addSequential(commandGroup {
                            this.addParallel(IntakeHoldCommand(), 0.001)
                            this.addParallel(MotionProfileCommand(Paths.CS_R_CENTER, true))
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

                "RS-LL" -> {
                    TODO("Drop in left scale, pick up cube and drop in left switch. This is the only scenario.")
                }
                "RS-LR" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH  -> TODO()
                        AutoMode.TWO_CUBE_SCALE -> TODO()
                    }
                }
                "RS-RL" -> {
                    TODO("Drop in right switch, Pick up cube and go to other side of scale. This is the only scenario.")
                }
                "RS-RR" -> {
                    when (autoMode) {
                        AutoMode.TWO_CUBE_BOTH  -> TODO()
                        AutoMode.TWO_CUBE_SCALE -> TODO()
                    }
                }
                else -> TODO("Does not exist.")
            }
        }
    }
}


enum class Paths(private val filePath: String) {

    CS_L_SWITCH("CS-L/Switch"),
    CS_L_CENTER("CS-L/Center"),

    CS_R_SWITCH("CS-R/Switch"),
    CS_R_CENTER("CS-R/Center"),

    CS_STRAIGHT("CS/Straight"),
    CS_EXCHANGE("CS/Exchange"),


    LS_LL_SWITCH("LS-LL/Switch"),
    LS_LL_SCALE1("LS-LL/Scale 1"),
    LS_LL_SCALE2("LS-LL/Scale 2"),

    LS_RL_SCALE1("LS-RL/Scale 1"),
    LS_RL_SCALE2("LS-RL/Scale 2");


    val trajectoryLeft
        get() = loadTrajectory(filePath + " Left.csv")

    val trajectoryRight
        get() = loadTrajectory(filePath + " Right.csv")


    /**
     * Loads the trajectory from the specified file.
     * @param path The path of the file to read the data from.
     * @return A list of points on the trajectory to read the motion profile from.
     */
    private fun loadTrajectory(path: String): TrajectoryList {
        javaClass.classLoader.getResourceAsStream(path).use { stream ->
            return InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").map { it.trim() }
                return@map TrajectoryData(pointData[0].toDouble(), pointData[1].toDouble(), pointData[2].toInt())
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

typealias TrajectoryList = List<TrajectoryData>

/**
 * Stores trajectory data for each point along the trajectory.
 */
data class TrajectoryData(private val position: Double, private val velocity: Double, val duration: Int) {

    // Converts feet and feet/sec into rotations and rotations/sec.
    private val rotations = Maths.feetToRotations(position, DriveConstants.WHEEL_RADIUS)
    private val rpm = Maths.feetPerSecondToRPM(velocity, DriveConstants.WHEEL_RADIUS)

    // Converts rotations and rotations/sec to native units and native units/100 ms.
    var nativeUnits = Maths.rotationsToNativeUnits(rotations, DriveConstants.SENSOR_UNITS_PER_ROTATION)
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, DriveConstants.SENSOR_UNITS_PER_ROTATION)
}


