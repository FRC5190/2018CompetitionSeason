package frc.team5190.robot.navigation

import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import java.io.InputStreamReader


enum class NAVHelper(private val leftFilePath: String, private val rightFilePath: String) {

    LEFTS_LEFT("left/left_left.csv", "left/left_right.csv"),
    LEFTS_RIGHT("left/right_let.csv", "left/right_right.csv"),
    CENTERS_LEFT("center/left_left.csv", "center/left_right.csv"),
    CENTERS_RIGHT("center/right_left.csv", "center/right_right.csv"),
    RIGHTS_LEFT("right/left_left.csv", "right/left_right.csv"),
    RIGHTS_RIGHT("right/right_left.csv", "right/right_right.csv");

    val trajectoryLeft by lazy {
        javaClass.classLoader.getResourceAsStream(leftFilePath).use { stream ->
            return@lazy InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").map { it.trim() }
                return@map TrajectoryData(pointData[0].toDouble(), pointData[1].toDouble(), pointData[2].toLong())
            }
        }
    }

    val trajectoryRight by lazy {
        javaClass.classLoader.getResourceAsStream(rightFilePath).use { stream ->
            return@lazy InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").map { it.trim() }
                return@map TrajectoryData(pointData[0].toDouble(), pointData[1].toDouble(), pointData[2].toLong())
            }
        }
    }
}

typealias TrajectoryList = List<TrajectoryData>

data class TrajectoryData(private val position: Double, private val velocity: Double, val duration: Long) {
    val rotations = Maths.feetToRotations(position, Hardware.WHEEL_RADIUS.toDouble())
    val rpm = Maths.feetPerSecondToRPM(velocity, Hardware.WHEEL_RADIUS.toDouble())

    val nativeUnits = Maths.rotationsToNativeUnits(rotations, 1440.0)
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, 1440.0)
}


