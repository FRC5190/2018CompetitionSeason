/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.listener.Listener
import frc.team5190.robot.util.commandGroup
import openrio.powerup.MatchData

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        fun getAuto(startingPosition: StartingPositions, switchOwnedSide: MatchData.OwnedSide, scaleOwnedSide: MatchData.OwnedSide, target: String = "Switch"): CommandGroup {

            var folder = startingPosition.name.substring(0, 1).toUpperCase() + "S-" +
                    switchOwnedSide.name.substring(0, 1).toUpperCase() +
                    scaleOwnedSide.name.substring(0, 1).toUpperCase()

            if (folder[0] == 'C') folder = folder.substring(0, folder.length - 1)

            when (folder) {
                "LS-LL" -> {
                    val requestId = Listener.requestPath(folder, target)

                    return commandGroup {
                        this.addSequential(MotionProfileCommand(requestId))
                    }
                }
                "CS-L", "CS-R" -> {
                    val switchId = Listener.requestPath(folder, "Switch")
                    val centerId = Listener.requestPath(folder, "Center")

                    return commandGroup {
                        this.addSequential(MotionProfileCommand(switchId))
                        this.addSequential(MotionProfileCommand(centerId, true))
                    }
                }
                else -> TODO("Generate other paths.")
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
