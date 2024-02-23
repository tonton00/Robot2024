// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot.subsystems;


//import org.photonvision.targeting.MultiTargetPNPResult;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PNPResult;
import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonTargetSortMode;
//import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import ;

public class RobotVision extends SubsystemBase {
  //if the camera isnt detected might need to use get name instead ||PhotonCamera camera = new PhotonCamera(getName());||
  //Check vision UI for camera nickname and set that to camera name or camera wont be detected.  
     PhotonCamera camera = new PhotonCamera("");
 
  //for some reason *targets* wasn't recognised and I couldnt figure out what package it was under in photovision
      // int targetID = targets.getFiducialId();
  //TODO: figure out what libary targets are under then set target id to fiducialID()
     //made targetID so I can work on the method
     int targetID;
     boolean hasTargets;
     
 


public void onAprilTagDetection()
{
  if (hasTargets == true) 
  {
     System.out.print(targetID);
  }



}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
