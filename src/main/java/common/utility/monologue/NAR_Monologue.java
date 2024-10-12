package common.utility.monologue;


import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import common.core.misc.NAR_Robot;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import monologue.LogLevel;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

import static monologue.LogLevel.*;

import java.util.ArrayList;
import java.util.List;

public class NAR_Monologue{ 
    static {
        NAR_Robot.addPeriodic(NAR_Monologue::update, 0.02);
    }
    /** 
     * Creates a NAR_Monologue
     * 
     * @param loggable Name of the class is logged. Must implement Logged.
     * @param rootpath The rootpath of what is logged
     * @param fileOnly Shuts on or off NetworkTables broadcasting for logged calls
     * @param lazyLogging When on, will not repeat two logged calls if they are equal in value
     */
    public NAR_Monologue(Logged loggable, String rootpath, boolean fileOnly, boolean lazyLogging){
        Monologue.setupMonologue(loggable, rootpath, fileOnly, lazyLogging);
    }

    public static void update(){

    }
}
