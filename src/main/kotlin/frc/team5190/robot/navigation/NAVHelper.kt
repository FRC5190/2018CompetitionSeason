package frc.team5190.robot.navigation

import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import java.io.InputStreamReader


enum class NAVHelper(private val leftFilePath: String, private val rightFilePath: String) {

    LEFT("", ""),
    RIGHT("", ""),
    CENTER("testpath_left.csv", "testpath_right.csv");

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

    val nativeUnits = Maths.rotationsToNativeUnits(rotations, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
    val nativeUnitsPer100Ms = Maths.rpmToNativeUnitsPer100Ms(rpm, Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
}


