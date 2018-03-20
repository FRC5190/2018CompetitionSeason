/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

@file:Suppress("DEPRECATION")

package frc.team5190.robot.auto

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import java.io.File

/**
 * Class that loads files from local resources or from Network Tables
 */
object Pathreader {

    private val allPaths: Map<String, Trajectory>

    init {
        allPaths = File("/home/lvuser/paths/").listFiles().filter { it.isDirectory }.map { folder ->
            folder.listFiles().filter { it.isFile }.map { file ->
                Pair("${folder.name}/${file.nameWithoutExtension}",
                        Pathfinder.readFromCSV(file))
            }
        }.flatten().toMap()
    }

    fun getPath(folderName: String, fileName: String): Trajectory? {
        return allPaths["$folderName/$fileName"]
    }
}
