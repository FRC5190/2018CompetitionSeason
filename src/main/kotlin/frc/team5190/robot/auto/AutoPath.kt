package frc.team5190.robot.auto

import java.io.InputStreamReader

enum class AutoPath(private val fileName: String) {
    LEFT(""),
    RIGHT(""),
    CENTER("testpath");

    val trajectoryLeft by lazy {
        loadTrajectoryFile("${fileName}_left.csv")
    }

    val trajectoryRight by lazy {
        loadTrajectoryFile("${fileName}_right.csv")
    }

    private fun loadTrajectoryFile(filePath: String): TrajectoryList {
        javaClass.classLoader.getResourceAsStream(filePath).use { stream ->
            return InputStreamReader(stream).readLines().map {
                val pointData = it.split(",").mapNotNull { it.toDoubleOrNull() }
                return@map TrajectoryData(pointData[0], pointData[1])
            }
        }
    }
}

typealias TrajectoryList = List<TrajectoryData>

data class TrajectoryData(val position: Double, val velocity: Double)