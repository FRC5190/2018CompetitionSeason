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
    LEFTS_LEFT("left/left_left", "left/left_right"),
//    LEFTS_RIGHT("left/right_left", "left/right_right"),
    CENTERS_LEFT("center/left_left", "center/left_right"),
//    CENTERS_RIGHT("center/right_left", "center/right_right"),
    RIGHTS_LEFT("right/left_left", "right/left_right");
//    RIGHTS_RIGHT("right/right_left", "right/right_right");

    var trajectoryLeft = loadTrajectory(leftFilePath + ".csv")

    var trajectoryRight = loadTrajectory(rightFilePath + ".csv")

    var trajectoryLeftDetailed = loadDetailedTrajectory(leftFilePath + "_detailed.csv")

    var trajectoryRightDetailed = loadDetailedTrajectory(rightFilePath + "_detailed.csv")


    /**
     * Loads the trajectory from the specified file.
     * @param path The path of the file to read the data from.
     * @return A list of points on the trajectory to read the motion profile from.
     */
    private fun loadTrajectory(path: String): TrajectoryList {
        println(path)
        javaClass.classLoader.getResourceAsStream(path).use { stream ->
            return InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").map { it.trim() }
                return@map TrajectoryData(pointData[0].toDouble(), pointData[1].toDouble(), pointData[2].toInt())
            }.toMutableList()
        }
    }

    private fun loadDetailedTrajectory(path: String): DetailedTrajectoryList {
        javaClass.classLoader.getResourceAsStream(path).use { stream ->
            return InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").map { it.trim() }
                return@map DetailedTrajectoryData(pointData[0].toDouble(), pointData[1].toDouble(), pointData[2].toDouble(), pointData[3].toDouble(),
                        pointData[4].toDouble(), pointData[5].toDouble(), pointData[6].toDouble(), pointData[7].toDouble())
            }.toMutableList()
        }
    }
}

typealias TrajectoryList = MutableList<TrajectoryData>
typealias DetailedTrajectoryList = MutableList<DetailedTrajectoryData>
typealias CombinedTrajectoryLists = MutableList<TrajectoryList>

/**
 * Stores trajectory data for each point along the trajectory.
 */
data class TrajectoryData(var position: Double, var velocity: Double, var duration: Int) {

    // Converts feet and feet/sec into rotations and rotations/sec.
    private val rotations = Maths.feetToRotations(position, Hardware.WHEEL_RADIUS)
    private val rpm = Maths.feetPerSecondToRPM(velocity, Hardware.WHEEL_RADIUS)

    // Converts rotations and rotations/sec to native units and native units/100 ms.
    var nativeUnits = Maths.rotationsToNativeUnits(rotations, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
}

data class DetailedTrajectoryData(private val dt: Double, val x: Double, val y: Double, private val pos: Double,
                                  private val vel: Double, private val accel: Double, private val jerk: Double, private val heading: Double) {
    val nativeUnits = Maths.rotationsToNativeUnits(Maths.feetToRotations(pos, Hardware.WHEEL_RADIUS), Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val h = heading
}


