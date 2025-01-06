// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** 
   * Constants related to operator controls and human interface devices.
   * USB ports are numbered 0-5 on the Driver Station
   */
  public static class OperatorConstants {
    /** USB port number for main driver controller (listed in Driver Station) */
    public static final int kDriverControllerPort = 0;
  }

  // Consider adding these categories:
  // public static class DriveConstants { ... }
  // public static class AutoConstants { ... }
  // public static class VisionConstants { ... }
}
