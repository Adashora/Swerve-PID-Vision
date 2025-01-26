// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
      public static final int mod0_Cancoder = 0;
      public static final Rotation2d mod0_CancoderOffset = Rotation2d.fromDegrees(0.0);

    }
    public static class mod1{

      public static final int drive_ID = 0;
      public static final int turn_ID = 0;
      public static final int mod1_Cancoder = 0;
      public static final Rotation2d mod1_CancoderOffset = Rotation2d.fromDegrees(0.0);


      
    }
    public static class mod2{

      public static final int drive_ID = 0;
      public static final int turn_ID = 0;
      public static final int mod2_Cancoder = 0;
      public static final Rotation2d mod2_CancoderOffset = Rotation2d.fromDegrees(0.0);



      
    }
    public static class mod3{


      public static final int drive_ID = 0;
      public static final int turn_ID = 0;
      public static final int mod3_Cancoder = 0;
      public static final Rotation2d mod3_CancoderOffset = Rotation2d.fromDegrees(0.0);


    }

  }

  

      public static final double Turn_KP = 0.0;
      public static final double Turn_KI = 0.0;
      public static final double Turn_KD = 0.0;

      public static final double Drive_KP = 0.1;
      public static final double Drive_KI = 0.2;
      public static final double Drive_KD = 0.3;

      public static double gyro_offset = 0.0;

      public static final double driveMotorRatio = 0;
      public static final double turningMotorRatio =0 / 7;
    
      //Conversion Factors
      public static final double driveMotorPosFactor = Math.PI * driveMotorRatio * 0;
      public static final double driveMotorVelFactor = driveMotorPosFactor / 60;
      public static final double turningMotorPosFactor = 360 / turningMotorRatio;
    
      
      //Max drive of the robot (in meters per second or radians per second)
      public static final double maxSpeed = 5;
      public static final double maxAutoSpeed = 2.5;
      public static final double maxAutoAcceleration = 0.5;
      public static final double maxAngularVelocity = 7.0;
      public static final double maxAutoAngularVelocity = Math.PI;

      public static final double robotLength = Units.inchesToMeters(23);
      public static final double robotWidth = Units.inchesToMeters(23);

    
  

  public static final SwerveDriveKinematics SwerveMap = new SwerveDriveKinematics(
    new Translation2d(robotLength / 2, robotWidth / 2), //++
    new Translation2d(robotLength / 2, -robotWidth /2),  //+-
    new Translation2d(-robotLength / 2, robotWidth / 2), //-+
    new Translation2d(-robotLength / 2, -robotWidth / 2) //--

    );

  

    //Intake Constants

    public static final int intakeMotorID = 44;

    public static final double intakeMotorSpeed = 1;

        //Pivot Angles -3

        public static final double intakePos = 73; //102 original
        public static final double subwooferPos = intakePos+15.5;
        public static final double ampPos = intakePos+39;
        public static final double podiumPos = intakePos+8.5;
        public static final double sourcePos = intakePos+10;
        public static final double backshotPos = intakePos+39.75;


        //test values
        // public static final double intakePos = 103.5;
        // public static final double subwooferPos = 118;
        // public static final double ampPos = 141.5;
        // public static final double podiumPos = 114.5;
        // public static final double sourcePos = 113;
        // public static final double backshotPos = 142.5;


    //Shooter Constants

    public static final double alignSpeed = 0.25;

    public static final int fastShooterMotorID = 41;
    public static final int slowShooterMotorID = 35;
    public static final int shooterHoldMotorID = 38;

    public static final int shooterPotID = 1;


    public static final double shooterIntakeSpeed = 0.5;

    public static final double shooterHighSpeed = 0.8;
    public static final double shooterLowSpeed = 0.2;
    public static final double shooterBumpSpeed = 0.2;

    public static final double shooterTargetVelocity = -4400;


    //Vision Constants

    public static final double cameraFOV = 42; //camera FOV in degrees
    public static final double cameraXRez = 640; //camera horizontal resolution in pixels
    public static final double cameraFOVRatio = cameraXRez / cameraFOV; //camera FOV in pixels

    public static final double alignkP = 0.2;

    public static final int speakerTagID = 1;



     


    //Pivot Constants

    public static final int pivotMotorID = 47;

    //test values
    // public static final double pivotkP = 1.5;
    // public static final double pivotkI = .2;
    // public static final double pivotkD = 0.6;

  //match values

    // public static final double pivotkP = .007; 
    // public static final double pivotkI = 0;
    // public static final double pivotkD = 0.002;

    public static final double pivotkP = 1.1; 
    public static final double pivotkI = 0.2;
    public static final double pivotkD = 0.05;


    public static final double pivotZeroValue = 177.2;
    public static final double pivotNinetyValue = 133.7;

    public static final double pivotPotToDegrees = (177.2-133.7)/90;




    //Climber Constants

    public static final int climberMotorLeftID = 40;
    public static final int climberMotorRightID = 29;

    public static final double climberUpSpeed = 0.5;
    public static final double climberSlowSpeed = 0.5;
    public static final double climberDownSpeed = 0.5;

    //

    //public static final int climberLeftServoID = 0;
    //public static final int climberRightServoID = 1;


  //auto constants
  //change PID for testing swerve auto

    public static final double autoRotateP = 0.25;
    public static final double autoRotateI = 0;
    public static final double autoRotateD = 0;

    public static final double autoTurningP = 0.5;
    public static final double autoTurningI = 0;
    public static final double autoTurningD = 0;

    public static final double autoXP = 0.5;
    public static final double autoXI = 0;
    public static final double autoXD = 0;

    public static final double autoYP = 0.5;
    public static final double autoYI = 0;
    public static final double autoYD = 0;

    public static final TrapezoidProfile.Constraints autoTurnController = new TrapezoidProfile.Constraints(maxAutoAngularVelocity, maxAutoAngularVelocity);

    //feedforward constants
  public static final double ffkS = 0.99;
  public static final double ffkV = 0.5; //5 m/s 2.44
  public static final double ffkA = 0.1; //accel to 5 m/s 0.27



  public static class vision {
    //limelight FOV 80x54
    //x
    public static final double width = 80;
    //y
    public static final double height = 54;
    //max voltage for horizontal allignment
    public static final double max_speed = maxSpeed / 4;
    public static final double kp = 2 / width * max_speed;
    public static final double ki = 0;
    public static final double kd = 0;
  }





}



