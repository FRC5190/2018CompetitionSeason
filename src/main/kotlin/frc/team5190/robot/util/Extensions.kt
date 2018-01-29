package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.LimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.CommandGroup

fun commandGroup(create: CommandGroup.() -> Unit): CommandGroup{
    val group = CommandGroup()
    create.invoke(group)
    return group
}

/**
 * Configures the PID for the specified motor
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Differential gain
 * @param power Max throttle power
 * @param rpm Max RPM
 * @param sensorUnitsPerRotation Sensor units per rotation
 * @param dev Feedback device used with the motor
 */
fun TalonSRX.configPIDF(slotIdx: Int, p: Double, i: Double, d: Double, power: Double, rpm: Double, sensorUnitsPerRotation: Double) {
    config_kP(slotIdx, p, 10)
    config_kI(slotIdx, i, 10)
    config_kD(slotIdx, d, 10)
    config_kF(slotIdx, Maths.calculateFGain(power, rpm, sensorUnitsPerRotation), 10)
}

fun TalonSRX.configPID(slotIdx: Int, p: Double, i: Double, d: Double, timeoutMs: Int) {
    config_kP(slotIdx, p, timeoutMs)
    config_kI(slotIdx, i, timeoutMs)
    config_kD(slotIdx, d, timeoutMs)
}

fun TalonSRX.configNominalOutput(percentForward: Double, percentReverse: Double, timeoutMs: Int) {
    configNominalOutputForward(percentForward, timeoutMs)
    configNominalOutputReverse(percentReverse, timeoutMs)
}

fun TalonSRX.configPeakOutput(percentForward: Double, percentReverse: Double, timeoutMs: Int) {
    configPeakOutputForward(percentForward, timeoutMs)
    configPeakOutputReverse(percentReverse, timeoutMs)
}

fun TalonSRX.configLimitSwitchSource(type: LimitSwitchSource, normalOpenOrClose: LimitSwitchNormal, timeoutMs: Int) {
    configForwardLimitSwitchSource(type, normalOpenOrClose, timeoutMs)
    configReverseLimitSwitchSource(type, normalOpenOrClose, timeoutMs)
}