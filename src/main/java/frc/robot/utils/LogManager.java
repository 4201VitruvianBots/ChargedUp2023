// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.UTIL;
import frc.robot.commands.util.DeleteAllLogs;
import frc.robot.utils.logging.LoggingUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LogManager {
  private final String mainPath = new File("").getAbsolutePath();

  private final String tempPath = "/tmp/";

  private ArrayList<String> logFilePaths = new ArrayList<>();

  public LogManager() {
    SmartDashboard.putData("Delete All Logs", new DeleteAllLogs(this));
  }

  public ArrayList<String> getLogFilePaths() {
    ArrayList<String> filePaths = new ArrayList<>();

    File folder = new File(mainPath);
    try {
      File[] allFiles = folder.listFiles();

      assert allFiles != null;
      for (File file : allFiles) {
        if (file.isFile() && file.getName().contains("wpilog")) {
          filePaths.add(file.getAbsolutePath());
        }
      }
    } catch (Exception ignored) {

    }

    return filePaths;
  }

  // WIP
  public void autoDeleteLogs(double secondsSinceModify) {
    for (String file : logFilePaths) {
      BasicFileAttributes attr;
      try {
        attr = Files.readAttributes(Paths.get(file), BasicFileAttributes.class);
        if (Math.abs(attr.lastModifiedTime().toMillis() - System.currentTimeMillis())
            > secondsSinceModify) {
          File fileObj = new File(file);
          if (fileObj.delete()) {
            System.out.println("Successfully deleted log file " + fileObj.getName());
          } else {
            System.out.println("Failed to delete log file " + fileObj.getName());
          }
        }
      } catch (IOException e) {
        //        e.printStackTrace();
      }
    }
  }

  public void deleteAllLogs() {
    for (String file : logFilePaths) {
      File fileObj = new File(file);
      if (fileObj.delete()) {
        System.out.println("Successfully deleted log file " + fileObj.getName());
      } else {
        System.out.println("Failed to delete log file " + fileObj.getName());
      }
    }
  }

  // Checks if the initialize file exists under the temporary folder.
  public boolean initTempExists() {
    try {
      return Files.isDirectory(Paths.get(tempPath))
          && Files.isRegularFile(Paths.get(tempPath + UTIL.tempFileName));
    } catch (Exception e) {
      return false;
    }
  }

  public void createInitTempFile() {
    try {
      if (!System.getProperty("os.name").equals("Windows")) {
        File fileObj = new File(tempPath + UTIL.tempFileName);
        fileObj.createNewFile();
      }
    } catch (IOException e) {
      //      DriverStation.reportWarning("Failed to create init file", e.getStackTrace());
    }
  }

  public void updateLogFilePaths() {
    logFilePaths = getLogFilePaths();
    // deleteAllLogs();
  }
}
