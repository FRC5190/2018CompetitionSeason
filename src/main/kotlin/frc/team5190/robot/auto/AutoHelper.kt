/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import java.io.InputStreamReader

/**
 * A class that contains information about the paths that the robot will take during autonomous.
 * @param leftFilePath The file path for the trajectory of the left side of the DriveTrain
 * @param rightFilePath The file path for the trajectory of the right side of the DriveTrain
 */
enum class AutoHelper(private val leftFilePath: String, private val rightFilePath: String) {

    // Various enums that represent the different paths the robot will take.
    LEFTS_LEFT("left/left_left.csv", "left/left_right.csv"),
    LEFTS_RIGHT("left/right_let.csv", "left/right_right.csv"),
    CENTERS_LEFT("center/left_left.csv", "center/left_right.csv"),
    CENTERS_RIGHT("center/right_left.csv", "center/right_right.csv"),
    RIGHTS_LEFT("right/left_left.csv", "right/left_right.csv"),
    RIGHTS_RIGHT("right/right_left.csv", "right/right_right.csv");

    val trajectoryLeft
        get() = loadTrajectory(leftFilePath)

    val trajectoryRight
        get() = loadTrajectory(rightFilePath)


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

typealias TrajectoryList = List<TrajectoryData>

/**
 * Stores trajectory data for each point along the trajectory.
 */
data class TrajectoryData(private val position: Double, private val velocity: Double, val duration: Int) {

    // Converts feet and feet/sec into rotations and rotations/sec.
    val rotations = Maths.feetToRotations(position, Hardware.WHEEL_RADIUS)
    val rpm = Maths.feetPerSecondToRPM(velocity, Hardware.WHEEL_RADIUS)

    // Converts rotations and rotations/sec to native units and native units/100 ms.
    var nativeUnits = Maths.rotationsToNativeUnits(rotations, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
}


