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
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide): CommandGroup {

            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL" -> {
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE))
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
                        this.addSequential(IntakeCommand(IntakeDirection.OUT, timeout = 0.3, outSpeed = 0.6))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        // 2nd cube
                        this.addSequential(MotionMagicCommand(-1.0))
                        this.addSequential(commandGroup {
                            this.addParallel(TurnCommand(170.0))
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                            this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                        })
                        this.addSequential(IntakeCommand(IntakeDirection.IN))
                        this.addSequential(MotionMagicCommand(7.11))
                        //this.addSequential(MotionMagicCommand(3.0))
                        //this.addSequential(FindCubeCommand())
                        //this.addSequential(MotionMagicCommand((VisionSubsystem.tgtRange_in - 10).coerceAtLeast(0.0) / 12))
                        this.addSequential(IntakeHoldCommand(), 0.001)
                        this.addSequential(commandGroup {
                            this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                            this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                        })
                        this.addSequential(MotionMagicCommand(1.0), 1.0)
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
                    return commandGroup {
                        this.addSequential(commandGroup {
                            this.addParallel(MotionProfileCommand(Paths.LS_LL_SCALE))
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

                        this.addSequential(MotionProfileCommand(Paths.FEET_1_5, true))
                    }
                }

                else -> TODO("Generate paths")
            }
        }
    }
}


enum class Paths(private val filePath: String) {

    CS_L_SWITCH("CS-L/Switch"),    // DONE
    CS_L_CENTER("CS-L/Center"),    // TODO Test

    CS_R_SWITCH("CS-R/Switch"),    // TODO Test
    CS_R_CENTER("CS-R/Center"),    // TODO Test

    CS_STRAIGHT("CS/Straight"),    // TODO Test
    CS_EXCHANGE("CS/Exchange"),    // TODO Test


    LS_LL_SWITCH("LS-LL/Switch"),   // TODO Test
    LS_LL_SWTOSC("LS-LL/SwToSc"),   // TODO Test
    LS_LL_SCALE("LS-LL/Scale"),
    LS_LL_SCALE_SECOND("LS-LL/Scale Second"),

    LS_LR_SWITCH("LS-LR/Switch"),
    LS_LR_SWTOSC("LS-LR/SwToSc"),   // TODO Generate

    LS_RL_SWITCH1("LS-RL/Switch 1"),  // TODO Test
    LS_RL_SWITCH2("LS-RL/Switch 2"),  // TODO Test
    LS_RL_SWITCH3("LS-RL/Switch 3"),
    LS_RL_SWTOSC("LS-RL/SwToSc"),   // TODO Generate

    LS_RR_SWITCH1("LS-RR/Switch1"),  // TODO Test
    LS_RR_SWITCH2("LS-RR/Switch2"),  // TODO Test
    LS_RR_SWTOSC("LS-RR/SwToSc"),   // TODO Generate


    RS_LL_SWITCH1("RS-LL/Switch1"),  // TODO Generate
    RS_LL_SWITCH2("RS-LL/Switch2"),  // TODO Generate
    RS_LL_SWTOSC("RS-LL/SwToSc"),   // TODO Generate

    RS_LR_SWITCH1("RS-LR/Switch1"),  // TODO Generate
    RS_LR_SWITCH2("RS-LR/Switch2"),  // TODO Generate
    RS_LR_SWTOSC("RS-LR/SwToSc"),   // TODO Generate

    RS_RL_SWITCH("RS-RL/Switch"),   // TODO Generate
    RS_RL_SWTOSC("RS-RL/SwToSc"),   // TODO Generate

    RS_RR_SWITCH("RS-RR/Switch"),   // TODO Generate
    RS_RR_SWTOSC("RS-RR/SwToSc"),   // TODO Generate


    FEET_1_5("Utils/1.5 Ft"),       // DONE
    FEET_10("Utils/10 Ft");         // DONE


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
