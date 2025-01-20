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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class dt{

    public static class mod0{

      public static final int drive_ID = 0;
      public static final int turn_ID = 0;

    }
    public static class mod1{


      
    }
    public static class mod2{


      
    }
    public static class mod3{


      
    }

  }

  public static class PID{

      public static final double Turn_KP = 0.0;
      public static final double Turn_KI = 0.0;
      public static final double Turn_KD = 0.0;

      public static final double Drive_KP = 0.0;
      public static final double Drive_KI = 0.0;
      public static final double Drive_KD = 0.0;

      public static final double Gyro_offset = 0.0;
  }




}
