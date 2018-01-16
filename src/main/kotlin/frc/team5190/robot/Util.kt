package frc.team5190.robot

class Maths {
    companion object {
        fun calculateFGain(power: Double, feetPerSecond: Double, wheelSize: Double, sensorUnitsPerRotation: Double): Double {
            return power * 1023 / (feetPerSecond * 12.0 / (wheelSize * Math.PI * 2.0) / 10.0 * sensorUnitsPerRotation)
        }

        fun calculateFGain(power: Double, rpm: Double, sensorUnitsPerRotation: Double): Double {
            return (power * 1023) / (rpm / 60.0 / 10 * sensorUnitsPerRotation)
        }

        fun feetToRotations(feetPerSecond: Double, wheelRadius: Double): Double {
            return feetPerSecond * 12.0 / (wheelRadius * Math.PI * 2.0)
        }

        fun feetPerSecondToRPM(feetPerSecond: Double, wheelRadius: Double): Double {
            return feetToRotations(feetPerSecond, wheelRadius) * 60f
        }
    }
}

fun main(args: Array<String>) {
    println(Maths.feetPerSecondToRPM(3.57, 2.0))
}