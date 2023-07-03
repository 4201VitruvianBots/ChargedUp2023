// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Keeps a log of how much memory the robot code is using. This is used by the GitHub action
 * MemoryLeakTest to test for memory leaks. Ran by declaring the environment variable
 * --insert-var-name-here on simulation call
 */
public class MemoryLog {
  private long totalMemory = 0;
  private long freeMemory = 0;
  private long usedMemory = 0;

  private long timeAtLastMemoryLog;
  private long timeSinceLastMemoryLog;

  private final String mainPath = new File("").getAbsolutePath();
  private final String logPath = mainPath + "/.github/artifacts/memorylog.txt";

  private final long startTime = System.nanoTime();

  public MemoryLog() {
    // Creates the file if it does not already exist
    try {
      File newFile = new File(logPath);
      newFile.createNewFile();
    } catch (IOException e) {
      //      e.printStackTrace();
    }

    try {
      final FileWriter clearFile = new FileWriter(logPath, false);
      clearFile.close();
    } catch (IOException clearFailed) {
      //      clearFailed.printStackTrace();
    }
  }

  // Throws an error that will be caught by the MemoryLeakTest action and cause it to fail
  private void throwError() {
    System.out.println("Too much memory being used, possible memory leak?");
    System.exit(1);
  }

  /* Forces garbage collection to run to keep memory usage data consistent
   * Reference: https://stackoverflow.com/a/74759/13137729 */
  private void garbageCollect() {
    Runtime.getRuntime().gc();
  }

  // Uses runtime functions to calculate total, free, and used memory
  private void updateMemoryValues() {
    this.totalMemory = Runtime.getRuntime().totalMemory();
    this.freeMemory = Runtime.getRuntime().freeMemory();
    this.usedMemory = this.totalMemory - this.freeMemory;
  }

  private void writeToLogFile(String string) {
    try {
      FileWriter logFile = new FileWriter(logPath, true);
      logFile.write(string);
      logFile.close();
    } catch (IOException writeFailed) {
      //      writeFailed.printStackTrace();
    }
  }

  // Run by RobotContainer once per scheduler run. Runs the methods defined above
  public void simulationPeriodic() {
    this.timeSinceLastMemoryLog = System.nanoTime() - this.timeAtLastMemoryLog;
    // Writes to the log file if at almost an hour of running
    if (timeSinceLastMemoryLog >= 60000000000L) { // Runs memory log every 60 seconds
      updateMemoryValues();

      // Prints the amount of megabytes being used by the robot code
      System.out.printf(
          "Amount of memory used by robot code: %04f MB\n", this.usedMemory / 1048576.0);
      // Checks if the robot code is using more than 192 MB of memory (3/4 of the roboRIO's memory)
      if (this.usedMemory / 1048576 > 192) {
        throwError();
      }

      // Resets the timer
      this.timeAtLastMemoryLog = System.nanoTime();

      // Writes to the log list
      writeToLogFile(
          String.format(
              "%04fs : %04f MB\n",
              (timeAtLastMemoryLog - startTime) / 1000000000.0, freeMemory / 1048576.0));
    } else if (timeSinceLastMemoryLog
        >= 57000000000L) { // Garbage collects 3 seconds before memory log
      garbageCollect();
    } else if (timeSinceLastMemoryLog
        >= 54000000000L) { // Garbage collects 6 seconds before memory log
      garbageCollect();
    }
  }
}
