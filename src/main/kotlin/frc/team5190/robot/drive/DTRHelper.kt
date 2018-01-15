@file:Suppress("unused")

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team5190.robot.util.Maths

class DTRHelper {
    companion object {
        fun configurePIDF(motor: TalonSRX, p: Double, i: Double, d: Double, power: Double, velocity: Double, wheelSize: Double, sensorUnitsPerRotation: Double, dev: FeedbackDevice) {
            configurePIDF(motor, p, i, d, Maths.calculateFGain(power, velocity, wheelSize, sensorUnitsPerRotation))
            motor.configSelectedFeedbackSensor(dev, 0, 10)
        }

        fun configurePIDF(motor: TalonSRX, p: Double, i: Double, d: Double, power: Double, rpm: Double, sensorUnitsPerRotation: Double, dev: FeedbackDevice) {
            configurePIDF(motor, p, i, d, Maths.calculateFGain(power, rpm, sensorUnitsPerRotation))
            motor.configSelectedFeedbackSensor(dev, 0, 10)
        }

        private fun configurePIDF(motor: TalonSRX, p: Double, i: Double, d: Double, f: Double) {
            motor.config_kP(0, p, 10)
            motor.config_kI(0, i, 10)
            motor.config_kD(0, d, 10)
            motor.config_kF(0, f, 10)
        }
    }
}