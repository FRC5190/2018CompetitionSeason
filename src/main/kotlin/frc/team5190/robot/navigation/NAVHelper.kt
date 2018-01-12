package frc.team5190.robot.navigation

import java.io.InputStreamReader

private val leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv"
private val rightPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv"

val numPoints = 54

lateinit var leftPoints: Array<Array<Double>>
lateinit var rightPoints: Array<Array<Double>>

enum class NAVHelper(private val leftFilePath: String, private val rightFilePath: String) {

    LEFT("", ""),
    RIGHT("", ""),
    CENTER(leftPath, rightPath);

    val trajectoryLeft by lazy {
        javaClass.classLoader.getResourceAsStream(leftFilePath).use { stream ->
            InputStreamReader(stream).readLines().map {
                it.split(",").mapNotNull { it.toDoubleOrNull() }.toTypedArray()
            }.toTypedArray()
        }
    }

    val trajectoryRight by lazy {
        javaClass.classLoader.getResourceAsStream(rightFilePath).use { stream ->
            InputStreamReader(stream).readLines().map {
                it.split(",").mapNotNull { it.toDoubleOrNull() }.toTypedArray()
            }.toTypedArray()
        }
    }
}


