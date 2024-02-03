// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.generated.TunerConstants;

public final class Constants {
    public static class Drive {
      public static final double kMaxDriveMeterS = TunerConstants.kSpeedAt12VoltsMps;
      public static final double kMaxAngularRadS = Math.PI; //rad/second
    }
}
