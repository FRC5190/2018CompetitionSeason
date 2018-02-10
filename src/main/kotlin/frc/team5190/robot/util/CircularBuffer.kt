package frc.team5190.robot.util

import java.util.*

class CircularBuffer(private val size: Int) {

    private val buffer: ArrayList<Double> = ArrayList(size)

    private var lowPeak: Int? = null
    private var highPeak: Int? = null
    private var peakDur: Int? = null
    private var iterator = 0

    private var numElements = 0
    private var sum = 0.0

    val average: Double
        get() {
            return if (numElements == 0)
                0.0
            else
                sum / numElements
        }

    var stallTime =  0.0

    val motorState: MotorState
        get () {
            if (lowPeak == null || highPeak == null)
                throw Exception("Did not configure values for Talon SRX.")

            iterator = if (average > highPeak!!) iterator + 1 else 0

            println(average)

            return when {
                average < lowPeak!! &&  iterator < peakDur!! / 20-> MotorState.GOOD
                average < highPeak!! && iterator < peakDur!! / 20 -> MotorState.OK
                average > highPeak!! && iterator < peakDur!! / 20 -> MotorState.OK
                else -> MotorState.STALL
            }
        }

    fun add(element: Double) {
        if (numElements > size - 1) {
            sum -= buffer[size - 1]
            buffer.removeAt(size - 1)
            numElements--
        }
        sum += element
        buffer.add(0, element)
        numElements++
    }

    fun configureForTalon(lowPeak: Int, highPeak: Int, peakDur: Int) {
        this.lowPeak = lowPeak
        this.highPeak = highPeak
        this.peakDur = peakDur
    }
}

enum class MotorState {
    GOOD, OK, STALL
}
