/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.drive.*
import frc.team5190.robot.elevator.ElevatorPreset
import frc.team5190.robot.elevator.ElevatorPresetCommand
import frc.team5190.robot.intake.*
import frc.team5190.robot.util.*
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
        fun getAuto(startingPositions: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, settings: Array<String>): CommandGroup {

            // Get the folder that the paths are contained within
            var folder = "${startingPositions.name.first()}S-${switchOwnedSide.name.first()}${scaleOwnedSide.name.first()}"
            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            val lsll = settings[0]
            val lslr = settings[1]
            val lsrl = settings[2]
            val lsrr = settings[3]

            return when (folder) {
                "CS-L", "CS-R" -> commandGroup {

                    val mpCommand = MotionProfileCommand(folder, "Switch", false, false)
                    var distance = 4.0

                    addSequential(commandGroup {
                        addParallel(mpCommand)
                        addParallel(commandGroup {
                            addSequential(TimedCommand(mpCommand.mpTime - 0.2))
                            addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                        })
                    })

                    addSequential(IntakeHoldCommand(), 0.001)
                    addSequential(AutoDriveCommand(-1.0), 0.7)

                    addSequential(commandGroup {
                        addParallel(TurnCommand(if (folder[folder.length - 1] == 'L') 90.0 else -90.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.INTAKE))
                    })

                    addSequential(object : AutoDriveCommand(distance) {
                        var initialDistance = 0.0

                        override fun initialize() {
                           initialDistance = (DriveSubsystem.falconDrive.leftEncoderPosition + DriveSubsystem.falconDrive.rightEncoderPosition) / 2.0
                        }

                        override fun end() {
                            distance = Maths.nativeUnitsToFeet(
                                    (((DriveSubsystem.falconDrive.leftEncoderPosition + DriveSubsystem.falconDrive.rightEncoderPosition) / 2.0)
                                            - initialDistance).toInt())
                        }

                        override fun isFinished(): Boolean {
                            return super.isFinished() || IntakeSubsystem.amperage > IntakeConstants.AMP_THRESHOLD
                        }
                    })

                    addSequential(AutoDriveCommand(-distance))

                    addSequential(commandGroup {
                        addParallel(TurnCommand(0.0))
                        addParallel(ElevatorPresetCommand(ElevatorPreset.SWITCH))
                    })

                    addSequential(IntakeCommand(IntakeDirection.OUT, outSpeed = 0.65, timeout = 0.65))
                }

                else -> throw IllegalArgumentException("Scenario does not exist.")
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
