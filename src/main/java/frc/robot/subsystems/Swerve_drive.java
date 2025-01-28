// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class Swerve_drive extends SubsystemBase {

  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator pose_estimator;
  private Field2d field;
  private Field2d field_pe;
  private LimelightHelpers.PoseEstimate limelighEstimate;
  private double yaw;
  private double [] xyz_dps;
  private boolean reject_updates;
  private final Swerve_Module[] dt;
  private final PigeonIMU gyro = new PigeonIMU(10);
  public final PIDController alignPID = new PIDController(Constants.alignkP, 0, 0);
  public final PIDController autoPID = new PIDController(Constants.autoRotateP, 0, 0);

  private final Joystick driverL;
  private final Joystick driverR; 


  
  
  // this code has more aura than matthew on skibidi
  
  /** Creates a new Swerve_drive. */
  public Swerve_drive(Joystick driverL, Joystick driverR) {

    this.dt = new Swerve_Module[] {
     new Swerve_Module
     (0, 
     Constants.dt.mod0.drive_ID, 
     Constants.dt.mod0.turn_ID, 
     Constants.dt.mod0.mod0_CancoderOffset, 
     Constants.dt.mod0.mod0_Cancoder),
     new Swerve_Module(1, 
     Constants.dt.mod1.drive_ID, 
     Constants.dt.mod1.turn_ID, 
     Constants.dt.mod1.mod1_CancoderOffset,       
     Constants.dt.mod1.mod1_Cancoder),
     new Swerve_Module(2,
      Constants.dt.mod2.drive_ID, 
      Constants.dt.mod2.turn_ID, 
      Constants.dt.mod2.mod2_CancoderOffset, 
      Constants.dt.mod2.mod2_Cancoder),
     new Swerve_Module(3, 
     Constants.dt.mod3.drive_ID, 
     Constants.dt.mod3.turn_ID, 
     Constants.dt.mod3.mod3_CancoderOffset, 
     Constants.dt.mod3.mod3_Cancoder)
    };

    odometry = new SwerveDriveOdometry(Constants.SwerveMap, getYaw(), //uses modules position to calculate robot position
    new SwerveModulePosition[] {dt[0].getposition(), dt[1].getposition(), dt[2].getposition(), dt[3].getposition()
    });

    pose_estimator = new SwerveDrivePoseEstimator(Constants.SwerveMap, getYaw(), new SwerveModulePosition[] {dt[0].getposition(),
      dt[1].getposition(), dt[2].getposition(), dt[3].getposition()}, new Pose2d());   //estimaties with vision data (havent added yet)



      this.driverL = driverL;
      this.driverR = driverR;

      Timer.delay(1);
      resetToAbsolute2();

      field = new Field2d();
      field_pe = new Field2d();

      xyz_dps = new double[3];
      
    }

    public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isAuto) {
      
      SmartDashboard.putNumber("Translation Angle", translation.getAngle().getDegrees());
    
      SwerveModuleState[] swerveModuleStates = Constants.SwerveMap.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

      for (Swerve_Module module : this.dt){
        module.set_desired_state(swerveModuleStates[module.module_number], isAuto);
      }
    
    }



    public Rotation2d getYaw() {
      
      yaw = gyro.getYaw() + Constants.gyro_offset;
  
      while (yaw > 360) {
        yaw = yaw - 360;
      }
      return Rotation2d.fromDegrees(yaw);
  
    }

    public void set_module_states(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);
      for (Swerve_Module module : this.dt){
        module.set_desired_state(desiredStates[module.module_number], false);
      } }
    

      public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Swerve_Module module : this.dt){
          states[module.module_number] = module.getState();
        }
        return states;
      }

      public void resetToAbsolute2(){
        for (Swerve_Module module : this.dt){
          module.resetToAbsolute();
        }
      }

      public void resetOdometry(Pose2d pose){

        odometry.resetPosition(getYaw(), 
        new SwerveModulePosition[] {dt[0].getposition(), dt[1].getposition(), dt[2].getposition(), dt[3].getposition()}, pose);
      }

      public Pose2d getPose(){
        return odometry.getPoseMeters();
      }

      public Pose2d get_pe_pose(){
        return pose_estimator.getEstimatedPosition();
      }

      public void Zero_gyro(double zero_pos){
        gyro.setYaw(zero_pos);
      }

      public double Yaw_rate(){

        gyro.getRawGyro(xyz_dps);
        return xyz_dps[2];
      }

    
    

  @Override
  public void periodic() {


    odometry.update(getYaw(), 
    new SwerveModulePosition[] {dt[0].getposition(), dt[1].getposition(), dt[2].getposition(), dt[3].getposition()});
    

    field.setRobotPose(getPose());

    pose_estimator.update(getYaw(),
     new SwerveModulePosition[] {dt[0].getposition(), dt[1].getposition(), dt[2].getposition(), dt[3].getposition()});
   
   LimelightHelpers.SetRobotOrientation("Limelight", pose_estimator.getEstimatedPosition().getRotation().getDegrees(), Yaw_rate(), 0, 0, 0, 0);
   
   if (Yaw_rate() > 720) {

reject_updates = true; }

else if (limelighEstimate.tagCount == 0) {

  reject_updates = true; }

  else {

    reject_updates = false; 
  } 
  if (reject_updates == false){

    pose_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.99999999));
    pose_estimator.addVisionMeasurement(limelighEstimate.pose, limelighEstimate.timestampSeconds);

  }
    SmartDashboard.putBoolean("Reject Updates", reject_updates);

    field_pe.setRobotPose(get_pe_pose());

    SmartDashboard.putData("Odometry Field", field);
    SmartDashboard.putData("Pose Estimator Field", field_pe);


    for(Swerve_Module module : dt){
      SmartDashboard.putNumber("mod" + module.module_number + "Cancoder", module.getCANCoder().getDegrees());
      SmartDashboard.putNumber("mod" + module.module_number + "integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber("mod" + module.module_number + "velocity", module.getState().speedMetersPerSecond);

      SmartDashboard.getNumber("gyro Yaw", getYaw().getDegrees()%360);
      SmartDashboard.getNumber("Joystick Hat", this.driverL.getPOV());
    }






}



   }
   
     // This method will be called once per scheduler run
  

