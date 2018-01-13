package frc.team5190.robot.util

class Maths {
    companion object {
        fun calculateFGain(power: Double, feetPerSecond: Double, wheelSize: Double, sensorUnitsPerRotation: Double): Double {
            return power * 1023 / (feetPerSecond * 12.0 / (wheelSize * Math.PI * 2.0) / 10.0 * sensorUnitsPerRotation)
        }

        fun calculateFGain(power: Double, rpm: Double, sensorUnitsPerRotation: Double): Double {
            return (power * 1023) / (rpm / 60.0 / 10 * sensorUnitsPerRotation)
        }

        fun feetPerSecondToRPM(feetPerSecond: Double, wheelRadius: Float): Double {
            return feetPerSecond * 12.0f / (wheelRadius.toDouble() * Math.PI * 2.0) * 60f
        }

        fun feetToRotations(feet: Double, wheelRadius: Double): Double {
            return feet / (Math.PI * 2.0 * wheelRadius)
        }
    }

}