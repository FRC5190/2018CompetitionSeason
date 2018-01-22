/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

/**
 * Contains all the conversions that are required for the robot.
 */
class Maths {
    companion object {
        /**
         * Calculates F gain
         * @param power Full throttle power.
         * @param rpm Max RPM
         * @param sensorUnitsPerRotation Sensor Units per rotation
         * @return F gain
         */
        fun calculateFGain(power: Double, rpm: Double, sensorUnitsPerRotation: Double): Double {
            return (power * 1023) / (rpm / 60.0 / 10 * sensorUnitsPerRotation)
        }

        /**
         * Converts feet per second to RPM
         * @param feetPerSecond Feet per second
         * @param wheelRadius Wheel radius
         * @return RPM
         */
        fun feetPerSecondToRPM(feetPerSecond: Double, wheelRadius: Double): Double {
            return (feetPerSecond * 12.0f / (wheelRadius * Math.PI * 2.0)) * 60f
        }

        /**
         * Converts feet to rotations
         * @param feet Feet
         * @param wheelRadius Wheel radius
         * @return Rotations
         */
        fun feetToRotations(feet: Double, wheelRadius: Double): Double {
            return (feet * 12.0) / (Math.PI * 2.0 * wheelRadius)
        }

        /**
         * Converts RPM to Native Units per 100 ms
         * @param rpm RPM
         * @param sensorUnitsPerRotation Sensor units per rotation
         * @return Native units per 100 ms
         */
        fun rpmToNativeUnitsPer100Ms(rpm: Double, sensorUnitsPerRotation: Double): Double {
            return rpm * sensorUnitsPerRotation / 600
        }

        /**
         * Converts rotations to native units
         * @param rot rotations
         * @param sensorUnitsPerRotation Sensor units per rotation
         * @return Native units
         */
        fun rotationsToNativeUnits(rot: Double, sensorUnitsPerRotation: Double): Double {
            return rot * sensorUnitsPerRotation
        }

        /**
         * Converts native units per 100 ms to RPM
         * @param nativeUnitsPer100Ms Native units per 100 ms
         * @return RPM
         */
        fun nativeUnitsPer100MsToRPM(nativeUnitsPer100Ms: Int): Double {
            return nativeUnitsPer100Ms * 600.0 / Hardware.NATIVE_UNITS_PER_ROTATION
        }

        /**
         * Converts native units to feet
         * @param nativeUnits Native units
         * @return Feet
         */
        fun nativeUnitsToFeet(nativeUnits: Int): Double {
            return (nativeUnits /  Hardware.NATIVE_UNITS_PER_ROTATION).toDouble() * (2 * Math.PI * Hardware.WHEEL_RADIUS) / 12.0
        }
    }
}