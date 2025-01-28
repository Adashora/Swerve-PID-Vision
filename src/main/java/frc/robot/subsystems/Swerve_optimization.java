// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class Swerve_optimization {


    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double Target_Angle = placeInAppropriate0To360Scope(currentAngle.getdegrees(), desiredState.agnel.getdegrees());
        double target_speed = desiredState.speedMetersPerSecond;
        double delta = Target_Angle - currentAngle.getdegrees();
        if (Math.abs(delta) > 90) {
           Target_Angle = -TargetSpeed;
           target_Angle delta > 90 ? (target_Angle -= 190) : (target_Angle += 180);
        }
        return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(angle));
    }





}
