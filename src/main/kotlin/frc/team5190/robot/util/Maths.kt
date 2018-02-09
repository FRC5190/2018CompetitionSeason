/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

import kotlin.math.roundToInt

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
        fun calculateFGain(power: Double, rpm: Int, sensorUnitsPerRotation: Double): Double {
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
        fun rpmToNativeUnitsPer100Ms(rpm: Double, sensorUnitsPerRotation: Int): Double {
            return rpm * sensorUnitsPerRotation.toDouble() / 600.0
        }

        fun feetPerSecondToNativeUnitsPer100Ms(feet: Double, wheelRadius: Double, sensorUnitsPerRotation: Int): Double{
            return Maths.rpmToNativeUnitsPer100Ms(Maths.feetPerSecondToRPM(feet, wheelRadius), sensorUnitsPerRotation)
        }

        /**
         * Converts rotations to native units
         * @param rot rotations
         * @param sensorUnitsPerRotation Sensor units per rotation
         * @return Native units
         */
        fun rotationsToNativeUnits(rot: Double, sensorUnitsPerRotation: Int): Double {
            return rot * sensorUnitsPerRotation
        }

        /**
         * Converts native units per 100 ms to RPM
         * @param nativeUnitsPer100Ms Native units per 100 ms
         * @return RPM
         */
        fun nativeUnitsPer100MsToRPM(nativeUnitsPer100Ms: Int): Double {
            return nativeUnitsPer100Ms * 600.0 / DriveConstants.SENSOR_UNITS_PER_ROTATION
        }

        /**
         * Converts native units to feet
         * @param nativeUnits Native units
         * @return Feet
         */
        fun nativeUnitsToFeet(nativeUnits: Int): Double {
            return nativeUnitsToFeet(nativeUnits, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS)
        }


        /**
         * Converts native units to feet
         * @param nativeUnits Native units
         * @param nativeUnitsPerRotation Native units per rotation
         * @param wheelRadius Size in inches of the wheel
         * @return Feet
         */
        fun nativeUnitsToFeet(nativeUnits: Int, nativeUnitsPerRotation: Int, wheelRadius: Double): Double {
            return nativeUnits.toDouble() /  nativeUnitsPerRotation.toDouble() * (2.0 * Math.PI * wheelRadius) / 12.0
        }

        /**
         * Converts native units to feet
         * @param nativeUnits Native units
         * @param nativeUnitsPerRotation Native units per rotation
         * @param wheelRadius Size in inches of the wheel
         * @return Feet
         */
        fun feetToNativeUnits(feet: Double, nativeUnitsPerRotation: Int, wheelRadius: Double): Int {
            return (feet * 12.0 / (2.0 * Math.PI * wheelRadius) * nativeUnitsPerRotation.toDouble()).roundToInt()
        }
    }
}