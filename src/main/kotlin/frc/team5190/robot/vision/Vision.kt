package frc.team5190.robot.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import edu.wpi.cscore.VideoMode
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.SerialPort
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
        if (visionPort == null) {
            try {
                print("[Vision] Creating JeVois SerialPort...")
                visionPort = SerialPort(BAUD_RATE, SerialPort.Port.kUSB1)
                println(" success!")
            } catch (e: Exception) {
                visionPort = null
                println(" failed!")
            }
        }

        thread(name = "Vision") {
            while (!stopped) {
                try {
                    periodic()
                    sleep(20)
                } catch (e: Exception) {
                    sleep(50)
                }
            }
        }

        // we can create the streaming server only after we start reading data from the serial port
        try {
            print("[Vision] Starting stream capture...")
            CameraServer.getInstance().startAutomaticCapture(0).apply {
                setPixelFormat(VideoMode.PixelFormat.kYUYV)
                setResolution(320, 252)
                setFPS(15)
            }
            println(" success!")
        } catch (e: Exception) {
            println(" failed!")
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
        while (visionPort!!.bytesReceived > 0) {
            lastDataReceived = System.currentTimeMillis()
            val string = visionPort!!.readString()
//            println(string)
            val obj = gson.fromJson<VisionTemplate>(string)
            isTgtVisible = obj.Track
            tgtAngle = obj.Angle
            tgtRange = obj.Range
        }

        if (System.currentTimeMillis() - lastDataReceived > 500) {
            // no updates for more than 500ms => not tracking
            isTgtVisible = 0
            tgtAngle = 0
            tgtRange = 0
        }
    }
}