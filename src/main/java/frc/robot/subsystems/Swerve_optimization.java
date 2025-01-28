// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
/** Add your docs here. */
public class Swerve_optimization {


    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) { //skbidi is fake
        double Target_Angle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());
        double target_speed = desiredState.speedMetersPerSecond; //get the wanted speed from the desired state
        double delta = Target_Angle - currentAngle; //differencces between the current angle and the target angle
        if (Math.abs(delta) > 90) { //if the difference is greater than 90 degrees
           target_speed = -target_speed;
           Target_Angle = delta > 90 ? (Target_Angle -= 190) : (Target_Angle += 180); //swerve module wont roatte mroe than 90 degrees
        }
        return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(Target_Angle));
    }

    private static double placeInAppropriate0To360Scope (double scope_reference, double new_angle){ 

            double lower_bound;
            double upper_bound;
            double lower_offset = scope_reference % 360; //how far scope ref is from 360 720 etc.

            if(lower_offset >= 0){ //seeing if offset if already alligned or ahead of multiple of 360


                lower_bound = scope_reference - lower_offset;     //calculating new bounds
                upper_bound = scope_reference + (360 - lower_offset); 
            }
            else{ //if offset is behind a multiple of 360

                upper_bound = scope_reference - lower_offset;        // calculating new bounds
                lower_bound = scope_reference - (360 + lower_offset);
            }
            while (new_angle >upper_bound){ //if the new angle is greater than the upper bound

                new_angle -= 360;     //keep subtracting 360 until in range
            }
            while (new_angle < lower_bound){ //if the new angle is less than the lower bound

                new_angle += 360;       //keep adding 360 until in range    
            }
            if (new_angle - scope_reference > 180) { //if the difference between the new angle and the scope reference is greater than 180
                new_angle -= 360;  //subtract 360 to get shorter path
              } else if (new_angle - scope_reference < -180) { //if the difference between the new angle and the scope reference is less than -180
                new_angle += 360; //add 360 to get shorter path
              }
              return new_angle; 


    } 





}
