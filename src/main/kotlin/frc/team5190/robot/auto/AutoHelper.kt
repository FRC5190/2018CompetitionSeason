/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import openrio.powerup.MatchData
import java.io.InputStreamReader

/**
 * Contains methods that help with autonomous
 */
class AutoHelper {
    companion object {
        /**
         * Returns a path from the specified data.
         * @param startingPosition The starting position of the robot.
         * @param ownedSide The side which is owned by the robot's alliance.
         * @return The path that the robot should take.
         */
        fun getPathFromData(startingPosition: StartingPositions, ownedSide: MatchData.OwnedSide): Paths {
            when {
                startingPosition == StartingPositions.LEFT -> {
                    if (ownedSide == MatchData.OwnedSide.LEFT) {
                        return Paths.LEFT_STATION_LEFT_SWITCH
                    }
                    return Paths.LEFT_STATION_RIGHT_SWITCH
                }
                startingPosition == StartingPositions.CENTER -> {
                    if (ownedSide == MatchData.OwnedSide.LEFT) {
                        return Paths.CENTER_STATION_LEFT_SWITCH
                    }
                    return Paths.CENTER_STATION_RIGHT_SWITCH
                }
                ownedSide == MatchData.OwnedSide.LEFT -> return Paths.RIGHT_STATION_LEFT_SWITCH
                else -> return Paths.RIGHT_STATION_RIGHT_SWITCH
            }
        }
    }
}


/**
 * A class that contains information about the paths that the robot will take during autonomous.
 * @param leftFilePath The file path for the trajectory of the left side of the DriveTrain
 * @param rightFilePath The file path for the trajectory of the right side of the DriveTrain
 */
enum class Paths(private val filePath: String) {

    // Various enums that represent the different paths the robot will take.
    LEFT_STATION_LEFT_SWITCH("left_station/direct_left_switch"),
    LEFT_STATION_RIGHT_SWITCH("left_station/direct_right_switch"),
    CENTER_STATION_LEFT_SWITCH("center_station/direct_left_switch"),
    CENTER_STATION_RIGHT_SWITCH("center_station/direct_right_switch"),
    RIGHT_STATION_LEFT_SWITCH("right_station/direct_left_switch"),
    RIGHT_STATION_RIGHT_SWITCH("right_station/direct_right_switch"),
    TEST("testpath");

    val trajectoryLeft
        get() = loadTrajectory(filePath + "_left.csv")

    val trajectoryRight
        get() = loadTrajectory(filePath + "_right.csv")


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
