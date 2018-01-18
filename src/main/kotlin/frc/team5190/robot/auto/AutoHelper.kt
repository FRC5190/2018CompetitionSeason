/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import java.io.InputStreamReader

/**
 * A class that contains information about the paths that the robot will take during autonomous.
 * @param leftFilePath The file path for the trajectory of the left side of the DriveTrain
 * @param rightFilePath The file path for the trajectory of the right side of the DriveTrain
 */
enum class AutoHelper(private val leftFilePath: String, private val rightFilePath: String) {

    // Various enums that represent the different paths the robot will take.
    LEFTS_LEFT("left/left_left", "left/left_right"),
    LEFTS_RIGHT("left/right_let", "left/right_right"),
    CENTERS_LEFT("center/left_left", "center/left_right"),
    CENTERS_RIGHT("center/right_left", "center/right_right"),
    RIGHTS_LEFT("right/left_left", "right/left_right"),
    RIGHTS_RIGHT("right/right_left", "right/right_right");

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

    companion object {
        fun generateNewPaths(index: Int, helper: AutoHelper) {
            val posX = (helper.trajectoryLeftDetailed[index].x + helper.trajectoryRightDetailed[index].x) / 2.0
            val posY = (helper.trajectoryLeftDetailed[index].y + helper.trajectoryRightDetailed[index].y) / 2.0
            val posA = NavX.angle

            val finalX = 14.0
            val finalY = 23.0
            val finalA = 270.0

            val waypoints = arrayOfNulls<Waypoint>(2)

            waypoints[0] = Waypoint(posX, posY, Pathfinder.d2r(posA))
            waypoints[1] = Waypoint(7.0, 20.0, Pathfinder.d2r(90.0))
            waypoints[2] = Waypoint(12.0, 25.0, 0.0)
            waypoints[3] = Waypoint(finalX, finalY, Pathfinder.d2r(finalA))

            val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 20.0, 5.0, 3.0, 60.0)
            val trajectory = Pathfinder.generate(waypoints, config)

            val tankModifier = TankModifier(trajectory).modify(25.0/12.0)

            val left = tankModifier.leftTrajectory
            val right = tankModifier.rightTrajectory

            val newLeftList: TrajectoryList = mutableListOf()
            val newRightList: TrajectoryList = mutableListOf()

            for (i in 0 until left.length()) {
                val seg = left.get(i)
                newLeftList[i] = TrajectoryData(seg.position, seg.velocity, (seg.dt * 1000).toInt())
            }

            for (i in 0 until right.length()) {
                val seg = right.get(i)
                newRightList[i] = TrajectoryData(seg.position, seg.velocity, (seg.dt * 1000).toInt())
            }

            val combined: CombinedTrajectoryLists = mutableListOf()

            combined[0] = newLeftList
            combined[1] = newRightList

            newTrajectories = combined
        }

        var newTrajectories: CombinedTrajectoryLists? = null

    }
}

typealias TrajectoryList = MutableList<TrajectoryData>
typealias DetailedTrajectoryList = MutableList<DetailedTrajectoryData>
typealias CombinedTrajectoryLists = MutableList<TrajectoryList>

/**
 * Stores trajectory data for each point along the trajectory.
 */
data class TrajectoryData(private var position: Double, private var velocity: Double, var duration: Int) {

    // Converts feet and feet/sec into rotations and rotations/sec.
    private val rotations = Maths.feetToRotations(position, Hardware.WHEEL_RADIUS)
    private val rpm = Maths.feetPerSecondToRPM(velocity, Hardware.WHEEL_RADIUS)

    // Converts rotations and rotations/sec to native units and native units/100 ms.
    var nativeUnits = Maths.rotationsToNativeUnits(rotations, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
}

data class DetailedTrajectoryData(private val dt: Double, val x: Double, val y: Double, private val pos: Double,
                                  private val vel: Double, private val accel: Double, private val jerk: Double, private val heading: Double)


