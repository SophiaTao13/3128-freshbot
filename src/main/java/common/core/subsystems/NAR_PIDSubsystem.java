package common.core.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.core.controllers.ControllerBase;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.TrapController;
import common.utility.NAR_Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;

/**
 * A subsystem based off of {@link PIDSubsystem} 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_PIDSubsystem extends SubsystemBase implements Logged{

    /**
     * SystemsTest specifically for PIDSubsystems.
     */
    public class SetpointTest extends SystemsTest {
        @Log private final double timeOut;
        @Log private final Timer timer = new Timer();
        @Log private double prevTime;

        /**
         * Creates a setpoint test for the system.
         * @param testName Name of the test.
         * @param setpoint Setpoint for the system to try and reach.
         * @param plateau Time the subsystem has to hold the setpoint.
         * @param timeOut The time the system has to reach the setpoint.
         * @param index Which controller you are referring to
         */
        public SetpointTest(String testName, double setpoint, double plateau, double timeOut, int index) {
            super(testName, runOnce(()-> startPID(setpoint, index)).andThen(waitSeconds(timeOut)));
            this.timeOut = timeOut;
            passCondition = ()-> timer.hasElapsed(plateau);
            prevTime = 0;
        }

        @Override
        public void initialize() {
            super.initialize();
            timer.restart();
            prevTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            super.execute();
            for(int i = 0; i < controllers.length; i++){
                if (!atSetpoint(i)) timer.reset();

            }
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            NAR_Log.info(testName, "Expected Time: " + timeOut);
            NAR_Log.info(testName, "Actual Time: " + (Timer.getFPGATimestamp() - prevTime));
        }

        @Override
        public boolean isFinished() {
            return super.isFinished() || passCondition.getAsBoolean();
        }

        /**
         * Add the test to the a Subsystem test
         */
        public void add() {
            Tester.getInstance().addTest(getName(), this);
        }
    }

    protected final ControllerBase[] controllers;
    protected boolean enabled;
    protected BooleanSupplier debug;
    protected DoubleSupplier setpoint;
    private double min, max;
    private double safetyThresh;
    private Timer safetyTimer = new Timer();

    private double prevMeasurement;
    private double prevVelocity;
    private double updateTime;
    private Timer updateTimer = new Timer();

    private boolean shouldLog = false;
    /**
     * Creates a new PIDSubsystem.
     *
     * @param controllers the Controllers to use
     */
    public NAR_PIDSubsystem(ControllerBase... controllers) {
        this.controllers = controllers;
        for(int i = 0; i < controllers.length; i++){

            final int finali = i;
            controllers[i].setMeasurementSource(()-> getMeasurement());
            controllers[i].addOutput(output -> useOutput(output, getSetpoint(finali), finali));
        }
        min = Double.NEGATIVE_INFINITY;
        max = Double.POSITIVE_INFINITY;
        safetyThresh = 5;
        prevMeasurement = 0;
        prevVelocity = 0;
        updateTime = controllers[0].getPeriod();
        updateTimer.start();
    }

    @Override
    public void periodic() {
        if (enabled) {
            for(int i = 0; i < controllers.length; i++){
                controllers[i].useOutput();

                if (safetyTimer.hasElapsed(safetyThresh)) disable();
                if (atSetpoint(i)) safetyTimer.restart();
            }
        }

        if (!shouldLog) return;

        if (updateTimer.hasElapsed(updateTime)) {
            final double measurement = getMeasurement();
            final double velocity = (measurement - prevMeasurement) / updateTimer.get();
            final double acceleration = (velocity - prevVelocity) / updateTimer.get();
            NAR_Shuffleboard.addData(getName(), "1stDerivative", velocity, 1, 2);
            NAR_Shuffleboard.addData(getName(), "2ndDerivative", acceleration, 1, 3);
            prevMeasurement = measurement;
            prevVelocity = velocity;
            updateTimer.restart();
        }
    }
    /**
     * Initializes shuffleboard with debug elements for PID + FF values.
     */
    @Log
    public void initShuffleboard() {
        shouldLog = true;
        for(int i = 0; i < controllers.length; i++){
            final int finali = i;
            NAR_Shuffleboard.addSendable(getName() +"_"+  i, "PID_Controller #" + i, controllers[i], 0, 0);
            NAR_Shuffleboard.addData(getName()+ "_"+ i, "Setpoint", ()-> getSetpoint(finali), 0, 2);
            NAR_Shuffleboard.addData(getName()+ "_"+ i, "AtSetpoint", ()-> atSetpoint(finali), 0, 3);
        }
        

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> getMeasurement(), 1, 1);

        NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = NAR_Shuffleboard.getBoolean(getName(), "TOGGLE");
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);
        for(ControllerBase controller : controllers){
            controller.setkS(NAR_Shuffleboard.debug(getName(), "kS", controller.getkS(), 3, 0));
            controller.setkV(NAR_Shuffleboard.debug(getName(), "kV", controller.getkV(), 3, 1));
            controller.setkA(NAR_Shuffleboard.debug(getName(), "kA", controller.getkA(), 3, 2));
            controller.setkG(NAR_Shuffleboard.debug(getName(), "kG", controller.getkG(), 3, 3));

        }
        
    }
    
    public ControllerBase getController() {
        return getController(0);
    }
    /**
     * Returns the Controller object controlling the subsystem
     *
     * @return The Controller
     * @param index Which controller you are refering to
     */
    public ControllerBase getController(int index) {
        return controllers[index];
    }

    /**
     * Sets the amount of time between measurement logging to account for noisy measurements.
     * @param timeSeconds The time in seconds between each update.
     */
    public void setUpdateTime(double timeSeconds) {
        updateTime = timeSeconds;
    }

    /**
     * Sets the safetyThreshold to disable PID if setpoint is not reached
     * @param timeSeconds The time in seconds for the safety threshold
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
    }
    
    public void setTolerance(double positionTolerance) {
        for(int i = 0; i < controllers.length; i++){
            setTolerance(positionTolerance, i);
        }
    }
    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param indices To which controller you want to change tolerance
     */
    public void setTolerance(double positionTolerance, int... indices) {
        int[] tempIndex = indices;
        for(int index: tempIndex){
            controllers[index].setTolerance(positionTolerance);   
        }
    }

    /**
     * Sets constraints for the setpoint of the PID subsystem.
     * @param min The minimum setpoint for the subsystem
     * @param max The maximum setpoint for the subsystem
     */
    public void setConstraints(double min, double max) {
        this.min = min;
        this.max = max;
    }
    public void setkG_Function(DoubleSupplier kG_Function) {
        for(int i = 0; i < controllers.length; i++){
            setkG_Function(kG_Function, i);
        }
    }
    /**
     * Sets the function returning the value multiplied against kG
     * @param kG_Function the function multiplied to kG
     * @param indices To which controllers you want to set KG
     */
    public void setkG_Function(DoubleSupplier kG_Function, int... indices) {
        int[] tempIndex = indices;
        for(int index: tempIndex){
            controllers[index].setkG_Function(kG_Function);
        }
    }
    
    public void startPID(double setpoint) {
        for(int i = 0; i < controllers.length; i++){
            startPID(setpoint, i);
        }
    }
    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     * @param indices To which controller you want to start PID  
     */
    public void startPID(double setpoint, int... indices) {
        int[] tempIndex = indices;
        for(int index: tempIndex){
            enable(index);
            controllers[index].setSetpoint(MathUtil.clamp((debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpoint, min, max));
        }
        
    }


    public void startPID(double... setpoints) {
        double[] setpointsArray = setpoints;
        for(int i = 0; i < setpointsArray.length; i++){
            enable(i);
            controllers[i].setSetpoint(MathUtil.clamp((debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpointsArray[i], min, max));
        }
        
    }
    
    public void enableContinuousInput(double minimumInput, int maximumInput) {
        enableContinuousInput(minimumInput, maximumInput, 0);
    }
    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @param indices To which controller you wan to enable continuous input
     */
    public void enableContinuousInput(double minimumInput, int maximumInput, int... indices) {
        int[] tempIndex = indices;
        for(int index: tempIndex){
            controllers[index].enableContinuousInput(minimumInput, maximumInput);
        }
    }

    public double getSetpoint() {
        return getSetpoint(0);
    }
    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint
     * @param index To which controller you want the setpoint
     */
    public double getSetpoint(int index) {
        return controllers[index].getSetpoint();
    }

    public boolean atSetpoint(){
        return atSetpoint(0);
    }
    /**
     * Returns true if subsystem is at setpoint, false if not
     *
     * @return If subsystem is at setpoint
     * @param index To which controller you are refering to
     */
    public boolean atSetpoint(int index) {
        return controllers[index].atSetpoint();
    }

    /**
     * Uses the output from the PIDController.
     *
     * @param output The output of the PIDController
     * @param setpoint The setpoint of the controller.
     * @param index To which controller you are referring to
     */
    protected abstract void useOutput(double output, double setpoint, double index);

    protected abstract void useOutput(double output, double setpoint);

    /**
     * Returns the measurement of the process variable used by the PIDController.
     *
     * @return the measurement of the process variable
     */
    protected abstract double getMeasurement();

    public void enable() {
        enable(0);
    }
    /** Enables the PID control. Resets the controller. */
    public void enable(int index) {
        enabled = true;
        safetyTimer.restart();
        controllers[index].reset();
    }

    /** Disables the PID control. Sets output to zero.
     * @param indices 
     */
    public void disable(int... indices) {
        enabled = false;
        for(int index: indices){
            useOutput(0, 0, index);

        }
    }

    public void disable() {
        enabled = false;
        for(int i = 0; i < controllers.length; i++){
            useOutput(0, 0, i);

        }
    }

    /**
     * Return whether the subsystem is in debug mode.
     * @return Boolean, true being in debug, false being not in debug.
     */
    public boolean isDebug() {
        return debug.getAsBoolean();
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }
}
