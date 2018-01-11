package frc.team5190.robot.navigation

import java.io.InputStreamReader

class NAVHelper {
   companion object {

       private lateinit var leftPoints : Array<Array<Double>>
       private lateinit var rightPoints : Array<Array<Double>>

       private val leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv"
       private val rightPath = "C:\\Users\\prate\\Downloads\\testpath_right.csv"


       fun getTrajectory(mode : AutoMode) {
           when (mode) {
               AutoMode.LEFT -> {
                   // TODO
               }

               AutoMode.CENTER -> {
                   leftPoints = loadCSV(leftPath)
                   rightPoints = loadCSV(rightPath)
               }

               AutoMode.RIGHT -> {
                   // TODO
               }
           }
       }

       private fun loadCSV(file: String): Array<Array<Double>> {
           javaClass.classLoader.getResourceAsStream(file).use { stream ->
               return InputStreamReader(stream).readLines().map { it.split(",").mapNotNull { it.toDoubleOrNull() }.toTypedArray() }.toTypedArray()
           }
       }
   }

    enum class AutoMode {
        LEFT, CENTER, RIGHT
    }
}


