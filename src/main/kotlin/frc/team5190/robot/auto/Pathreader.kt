/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */


package frc.team5190.robot.auto

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import edu.wpi.first.networktables.*
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import java.io.File
import java.util.concurrent.*

/**
 * Class that loads files from local resources or from Network Tables
 */
object Pathreader {

    private const val PATHFEEDER_MODE = true

    private val allPaths = File("/home/lvuser/paths/").listFiles().filter { it.isDirectory }.map { folder ->
        folder.listFiles().filter { it.isFile }.map { file ->
            Pair("${folder.name}/${file.nameWithoutExtension}", Pathfinder.readFromCSV(file))
        }
    }.flatten().toMap()

    private val pathfinderOutputTable = NetworkTableInstance.getDefault().getTable("pathfinderOutput")!!
    private val pathfinderInputTable = NetworkTableInstance.getDefault().getTable("pathfinderInput")!!

    private var pathRequestId = 0L

    private val gson = Gson()

    fun getPath(folderName: String, fileName: String): Array<Trajectory> {
        @Suppress("ConstantConditionIf")
        if (PATHFEEDER_MODE) {
            println("Requesting path from pathfeeder...")
            val requestId = pathRequestId++
            val responseEntry = pathfinderOutputTable.getEntry("path_${requestId}_response")
            val requestEntry = pathfinderInputTable.getEntry("path_${requestId}_request")

            val requestFuture = CompletableFuture<Array<Trajectory>>()

            var listenerId = 0

            try {
                // Create response listener
                listenerId = responseEntry.addListener({ entryNotification: EntryNotification ->
                    requestFuture.complete(gson.fromJson(entryNotification.value.string))
                }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)

                // Send the request
                requestEntry.forceSetString(gson.toJson(PathRequest(folderName, fileName)))

                NetworkTableInstance.getDefault().flush()

                try {
                    // Wait for response for 4 seconds
                    return requestFuture.get(10L, TimeUnit.SECONDS)
                } catch (e: TimeoutException) {
                    println("Failed to get path from PathFeeder, loading from file instead")
                    e.printStackTrace()
                }
            } finally {
                // Clean up
                responseEntry.removeListener(listenerId)
                responseEntry.delete()
                requestEntry.delete()
            }
        }
        return arrayOf(allPaths["$folderName/$fileName Left Detailed"]!!, allPaths["$folderName/$fileName Right Detailed"]!!)
    }

    class PathRequest(val folderName: String, val fileName: String)

}
