package frc.team5190.robot.util

class Maths {
    companion object {
        fun calculateFGain(power: Double, feetPerSecond: Double, wheelSize: Double, sensorUnitsPerRotation: Double): Double {
            return power * 1023 / (feetPerSecond * 12.0 / (wheelSize * Math.PI * 2.0) / 10.0 * sensorUnitsPerRotation)
        }

        fun calculateFGain(power: Double, rpm: Double, sensorUnitsPerRotation: Double): Double {
            val f = (power * 1023) / (rpm / 60.0 / 10 * sensorUnitsPerRotation)
            println(f)
            return f
        }

        fun feetPerSecondToRPM(feetPerSecond: Double, wheelRadius: Double): Double {
            return (feetPerSecond * 12.0f / (wheelRadius * Math.PI * 2.0)) * 60f
        }

        fun feetToRotations(feet: Double, wheelRadius: Double): Double {
            return (feet * 12.0) / (Math.PI * 2.0 * wheelRadius)
        }

        fun rpmToNativeUnitsPer100Ms(rpm: Double, sensorUnitsPerRotation: Double): Double {
            return rpm * sensorUnitsPerRotation / 600
        }

        fun rotationsToNativeUnits(rot: Double, sensorUnitsPerRotation: Double): Double {
            return rot * sensorUnitsPerRotation
        }
    }
}