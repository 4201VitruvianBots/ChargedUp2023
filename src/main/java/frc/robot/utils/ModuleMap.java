package frc.robot.utils;

import java.util.*;

/**
 * Contains functions to convert {@link Map}s with {@link SwerveDriveModulePosition} keys to and from arrays so
 * that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {

  /**
   * Creates a {@code Map} with {@link SwerveDriveModulePosition} keys from multiple values, in the order
   * specified in the {@link SwerveDriveModulePosition} enum.
   *
   * <p>For processing the output of a WPILib swerve function which returns an array.
   *
   * @param values Must have at least as many elements as {@link SwerveDriveModulePosition} has entries. Any
   *     entries after will be ignored.
   */
  @SafeVarargs
  public static <V> Map<frc.robot.Constants.SwerveDriveModulePosition, V> of(V... values) {
    Map<frc.robot.Constants.SwerveDriveModulePosition, V> map = new HashMap<>();
    for (int i = 0; i < frc.robot.Constants.SwerveDriveModulePosition.values().length; i++) {
      map.put(frc.robot.Constants.SwerveDriveModulePosition.values()[i], values[i]);
    }
    return map;
  }

  /**
   * Returns the values from a map as a {@link List} in the same order as in the {@link
   * SwerveDriveModulePosition} enum.
   *
   * <p>You can use this in a for/in loop without needing to supply an empty array like in {@link
   * #orderedValues(Map, Object[]) orderedValues}.
   */
  public static <V> List<V> orderedValuesList(Map<frc.robot.Constants.SwerveDriveModulePosition, V> map) {
    ArrayList<V> list = new ArrayList<>();
    for (frc.robot.Constants.SwerveDriveModulePosition i : frc.robot.Constants.SwerveDriveModulePosition.values()) {
      list.add(map.get(i));
    }
    return list;
  }

  /**
   * Returns the values from the map as an {@code Array} in the same order as in the {@link
   * SwerveDriveModulePosition} enum.
   *
   * <p>Useful when a WPILib swerve function requires an array as input.
   *
   * @param array An array of the class to output an array of, e.g. {@code
   *     moduleTranslations.valuesArray(new Translation2d[0])}. Required because Java can't make an
   *     array of generics.
   */
  public static <V> V[] orderedValues(Map<frc.robot.Constants.SwerveDriveModulePosition, V> map, V[] array) {
    return orderedValuesList(map).toArray(array);
  }
}
