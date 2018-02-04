/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.elevator.AutoElevatorCommand
import frc.team5190.robot.elevator.ElevatorPosition
import frc.team5190.robot.intake.*
import frc.team5190.robot.util.*
import openrio.powerup.MatchData
import java.io.InputStreamReader

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        fun getCommandGroupFromData(startingPosition: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide): CommandGroup {
            return when (startingPosition) {
                StartingPositions.LEFT -> {
                    when (switchOwnedSide) {
                        MatchData.OwnedSide.LEFT -> {    // Left Switch
                            when (scaleOwnedSide) {
                                MatchData.OwnedSide.LEFT -> commandGroup {
                                    // LL
                                    this.addSequential(commandGroup {
                                        this.addParallel(MotionProfileCommand(Paths.LS_LL_SWITCH))
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                    })

                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.2))

                                    this.addSequential(commandGroup {
                                        this.addParallel(IntakeHoldCommand(), 0.001)
                                        this.addParallel(MotionProfileCommand(Paths.FEET_1_5, true))
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                        this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                    })

                                    this.addSequential(commandGroup {
                                        this.addParallel(MotionProfileCommand(Paths.FEET_1_5))
                                        this.addParallel(IntakeCommand(IntakeDirection.IN, true, 2.5))
                                    })

                                    this.addSequential(IntakeHoldCommand(), 0.001)
                                    this.addSequential(MotionProfileCommand(Paths.LS_LL_SWTOSC))

                                    this.addSequential(commandGroup {
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.BEHIND))
                                    })

                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.2))
                                    this.addSequential(IntakeHoldCommand(), 0.001)

                                }
                                MatchData.OwnedSide.RIGHT -> commandGroup {
                                    // LR
                                    this.addSequential(commandGroup {
                                        this.addParallel(MotionProfileCommand(Paths.LS_LR_SWITCH))
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                        this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                                    })

                                    this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.2))

                                    this.addSequential(commandGroup {
                                        this.addParallel(IntakeHoldCommand(), 0.001)
                                        this.addParallel(MotionProfileCommand(Paths.FEET_1_5, true))
                                        this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                        this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                    })

                                    this.addSequential(commandGroup {
                                        this.addParallel(MotionProfileCommand(Paths.FEET_1_5))
                                        this.addParallel(IntakeCommand(IntakeDirection.IN, true, 2.5))
                                    })

                                    this.addSequential(IntakeHoldCommand(), 0.001)
                                    this.addSequential(MotionProfileCommand(Paths.LS_LR_SWTOSC))
                                }

                                MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                            }
                        }
                        MatchData.OwnedSide.RIGHT -> {  // Right Switch
                            when (scaleOwnedSide) {
                                MatchData.OwnedSide.LEFT -> TODO()   // arrayListOf(Paths.LS_RL_SWITCH1, Paths.LS_RL_SWITCH2, Paths.LS_LL_SWTOSC)  // RL
                                MatchData.OwnedSide.RIGHT -> TODO()   // arrayListOf(Paths.LS_RR_SWITCH1, Paths.LS_RR_SWITCH2, Paths.LS_LR_SWTOSC)  // RR
                                MatchData.OwnedSide.UNKNOWN -> TODO()   // arrayListOf(Paths.CS_STRAIGHT)
                            }
                        }
                        MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                    }
                }

                StartingPositions.CENTER -> {
                    when (switchOwnedSide) {
                        MatchData.OwnedSide.LEFT -> commandGroup {
                            // L
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.CS_L_SWITCH))
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                            })

                            this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.5, 0.2))

                            this.addSequential(commandGroup {
                                this.addParallel(IntakeHoldCommand(), 0.001)
                                this.addParallel(MotionProfileCommand(Paths.CS_L_CENTER, true))
                            })
//
//                            this.addSequential(TurnCommand(1.0))  // TODO VISION
//
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.CS_STRAIGHT))
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                this.addParallel(IntakeCommand(IntakeDirection.IN, true, 2.0))
                            })
//
                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(TurnCommand(-140.0))
                            this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT))
                            this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.5))
                        }
                        MatchData.OwnedSide.RIGHT -> commandGroup {
                            // R
                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.CS_R_SWITCH))
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                                this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                            })

                            this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.5, 0.2))

                            this.addSequential(commandGroup {
                                this.addParallel(IntakeHoldCommand(), 0.001)
                                this.addParallel(MotionProfileCommand(Paths.CS_R_CENTER, true))
                            })

//                            this.addSequential(TurnCommand(0.0)) // TODO VISION

                            this.addSequential(commandGroup {
                                this.addParallel(MotionProfileCommand(Paths.CS_STRAIGHT))
                                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                                this.addParallel(IntakeCommand(IntakeDirection.IN, true, 4.0))
                            })

                            this.addSequential(IntakeHoldCommand(), 0.001)
                            this.addSequential(TurnCommand(-140.0))
                            this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT))
                            this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.5))
                        }
                        MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                    }
                }

                StartingPositions.RIGHT -> {
                    when (switchOwnedSide) {
                        MatchData.OwnedSide.LEFT -> {    // Left Switch
                            when (scaleOwnedSide) {
                                MatchData.OwnedSide.LEFT -> TODO() // arrayListOf(Paths.RS_LL_SWITCH1, Paths.RS_LL_SWITCH2, Paths.LS_LL_SWTOSC)  // LL
                                MatchData.OwnedSide.RIGHT -> TODO() //arrayListOf(Paths.RS_LR_SWITCH1, Paths.RS_LL_SWITCH2, Paths.LS_LR_SWTOSC)  // LR
                                MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                            }
                        }
                        MatchData.OwnedSide.RIGHT -> {    // Right Switch
                            when (scaleOwnedSide) {
                                MatchData.OwnedSide.LEFT -> TODO() //arrayListOf(Paths.RS_RL_SWITCH, Paths.LS_LL_SWTOSC)  // RL
                                MatchData.OwnedSide.RIGHT -> TODO() //arrayListOf(Paths.RS_RR_SWITCH, Paths.LS_LR_SWTOSC)  // RR
                                MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                            }
                        }
                        MatchData.OwnedSide.UNKNOWN -> commandGroup { this.addSequential(MotionProfileCommand(Paths.CS_STRAIGHT)) }
                    }
                }
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

    LS_LR_SWITCH("LS-LR/Switch"),
    LS_LR_SWTOSC("LS-LR/SwToSc"),   // TODO Generate

    LS_RL_SWITCH1("LS-RL/Switch1"),  // TODO Test
    LS_RL_SWITCH2("LS-RL/Switch2"),  // TODO Test
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
    private val rotations = Maths.feetToRotations(position, Hardware.WHEEL_RADIUS)
    private val rpm = Maths.feetPerSecondToRPM(velocity, Hardware.WHEEL_RADIUS)

    // Converts rotations and rotations/sec to native units and native units/100 ms.
    var nativeUnits = Maths.rotationsToNativeUnits(rotations, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
}
