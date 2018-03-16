package frc.team5190.robot.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import edu.wpi.cscore.VideoMode
import edu.wpi.first.wpilibj.*
import java.lang.Thread.sleep
import kotlin.concurrent.thread

object Vision {

    var isTgtVisible = 0L
    var tgtAngle = 0L
    var tgtRange = 0L
    private var visionPort: SerialPort? = null
    private var lastDataReceived: Long = 0
    private var stopped = false
    private const val BAUD_RATE = 115200

    init {
        try {
            println("[Vision] Creating JeVois SerialPort...")
            visionPort = SerialPort(BAUD_RATE, SerialPort.Port.kUSB1)
            println("[Vision] Success!")
        } catch (e: Exception) {
            visionPort = null
            println("[Vision] Failed!")
        }

        if (visionPort != null) {
            thread(name = "Vision") {
                while (!stopped) {
                    try {
                        periodic()
                        sleep(10)
                    } catch (e: Exception) {
                        sleep(50)
                    }
                }
            }

            // we can create the streaming server only after we start reading data from the serial port
            try {
                println("[Vision] Starting stream capture...")
                CameraServer.getInstance().startAutomaticCapture(0).apply {
                    setPixelFormat(VideoMode.PixelFormat.kYUYV)
                }
                println("[Vision] Success!...")
            } catch (e: Exception) {
                println("[Vision] Failed!")
            }
        }
    }

    fun stop() {
        stopped = true
    }

    private val gson = Gson()

    data class VisionTemplate(
            val Track: Long,
            val Angle: Long,
            val Range: Long)

    private fun periodic() {
        if (visionPort!!.bytesReceived > 0) {
            lastDataReceived = System.currentTimeMillis()
            val string = visionPort!!.readString()
            println(string)
            val obj = gson.fromJson<VisionTemplate>(string)
            isTgtVisible = obj.Track
            tgtAngle = obj.Angle
            tgtRange = obj.Range
        }
        else if (System.currentTimeMillis() - lastDataReceived > 200) {
            // no updates for more than 200ms => not tracking
            isTgtVisible = 0
            tgtAngle = 0
            tgtRange = 0
        }
    }
}