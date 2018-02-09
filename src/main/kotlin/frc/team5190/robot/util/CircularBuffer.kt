package frc.team5190.robot.util

import java.util.*

class CircularBuffer(private val size: Int) {

    private val buffer: ArrayList<Double> = ArrayList(size)

    private var peak: Int? = null
    private var peakDur: Int? = null
    private var iterator = 0

    private val numElements
        get() = buffer.size

    private var sum = 0.0

    val average
        get() = sum / numElements

    val motorState: MotorState
        get () {
            if (peak == null || peakDur == null)
                throw Exception("Did not configure values for Talon SRX.")

            iterator = if (average > peak!!) iterator + 1 else 0

            return if (iterator < peakDur!! / 20 && average < peak!!) {
                MotorState.OK
            } else if (iterator < peakDur!! / 20 && average > peak!!) {
                MotorState.STALL
            } else {
                MotorState.DEAD
            }
        }

    fun add(element: Double) {
        if (numElements > size - 1) {
            sum -= buffer[size - 1]
            buffer.removeAt(size - 1)
        }
        sum += element
        buffer.add(0, element)
    }

    fun configureForTalon(peak: Int, peakDur: Int) {
        this.peak = peak
        this.peakDur = peakDur
    }
}

enum class MotorState {
    OK, STALL, DEAD
}
