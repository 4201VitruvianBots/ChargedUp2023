package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;

public class ChargedUpNodeMask {
    private ArrayList<Pose2d> allNodes = new ArrayList<>();
    private ArrayList<Translation2d> validNodes = new ArrayList<>();

    private ArrayList<Translation2d> blueHybridNodes = new ArrayList<>();
    private ArrayList<Translation2d> blueMidConeNodes = new ArrayList<>();
    private ArrayList<Translation2d> blueMidCubeNodes = new ArrayList<>();
    private ArrayList<Translation2d> blueHighConeNodes = new ArrayList<>();
    private ArrayList<Translation2d> blueHighCubeNodes = new ArrayList<>();
    private ArrayList<Translation2d> blueCoopertitionNodes = new ArrayList<>();

    private ArrayList<Translation2d> blueNodes = new ArrayList<>();
    private ArrayList<Translation2d> redHybridNodes = new ArrayList<>();
    private ArrayList<Translation2d> redMidConeNodes = new ArrayList<>();
    private ArrayList<Translation2d> redMidCubeNodes = new ArrayList<>();
    private ArrayList<Translation2d> redHighConeNodes = new ArrayList<>();
    private ArrayList<Translation2d> redHighCubeNodes = new ArrayList<>();
    private ArrayList<Translation2d> redCoopertitionNodes = new ArrayList<>();

    private ArrayList<Translation2d> redNodes = new ArrayList<>();

    private ArrayList<Translation2d> ignoredNodes = new ArrayList<>();
}
