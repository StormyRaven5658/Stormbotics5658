/**
 * WPI Compliant motor controller class.
 * WPILIB's object model requires many interfaces to be implemented to use
 * the various features.
 * This includes...
 * - LiveWindow/Test mode features
 * - getRotation2d/Gyro Interface
 * - Simulation Hooks
 */

package com.ctre.phoenix.sensors;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.HAL;
import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;

/**
 * Pigeon 2 Class. Class supports communicating over CANbus.
 *
 * @deprecated This device's Phoenix 5 API is deprecated for removal in the
 * 2025 season. Users should update to Phoenix 6 firmware and migrate to the
 * Phoenix 6 API. A migration guide is available at
 * https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/index.html.
 * <p>
 * If the Phoenix 5 API must be used for this device, the device must have 22.X
 * firmware. This firmware is available in Tuner X after selecting Phoenix 5 in
 * the firmware year dropdown.
 */
@Deprecated(since = "2024", forRemoval = true)
public class WPI_Pigeon2 extends Pigeon2 {

    private double[] _xyz_dps; //instance this so we don't alloc when getRate is called.

    private SimDevice m_simPigeon;
    private SimDouble m_simYaw;
    private SimDouble m_simRawYaw;
    private DeviceType m_type;

    // callbacks to register
    private SimValueCallback onValueChangedCallback = new OnValueChangedCallback();
    private Runnable onPeriodicCallback = new OnPeriodicCallback();

    // returned registered callbacks
    private ArrayList<CallbackStore> simValueChangedCallbacks = new ArrayList<CallbackStore>();
    /**
     * Constructor for Pigeon 2.
     * @param deviceNumber device ID of Pigeon 2
     * @param canbus Name of the CANbus; can be a CANivore device name or serial number.
     *               Pass in nothing or "rio" to use the roboRIO.
     */
    public WPI_Pigeon2(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
        m_type = DeviceType.CANCoderType;
        SendableRegistry.addLW(this, "Pigeon 2 ", deviceNumber);
        init();
    }
    /**
     * Constructor for Pigeon 2.
     * @param deviceNumber device ID of Pigeon 2
     */
    public WPI_Pigeon2(int deviceNumber) {
        this(deviceNumber, "");
    }

    private void init(){
        _xyz_dps = new double[3];

        m_simPigeon = SimDevice.create("CANGyro:Pigeon 2", getDeviceID());
        if(m_simPigeon != null) {
            HAL.registerSimPeriodicBeforeCallback(onPeriodicCallback);

            m_simYaw = m_simPigeon.createDouble("yaw", Direction.kOutput, 0);

            m_simRawYaw = m_simPigeon.createDouble("rawYawInput", Direction.kInput, 0);

            SimDeviceSim sim = new SimDeviceSim("CANGyro:Pigeon 2");
            simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simRawYaw, onValueChangedCallback, true));
        }
    }

    public int getDeviceID() {
       
        throw new UnsupportedOperationException("Unimplemented method 'getDeviceID'");
    }
    // ----- Auto-Closable, from Gyro ----- //
    @Override
    public void close(){
        SendableRegistry.remove(this);
        if(m_simPigeon != null) {
            m_simPigeon.close();
            m_simPigeon = null;
        }
        super.close(); //Pigeon device, replace with super.close() once implemented
    }


    // ----- Callbacks for Sim ----- //
    private class OnValueChangedCallback implements SimValueCallback {
                @Override
                public void callback(String name, int handle, int direction, HALValue value) {
                        String deviceName = SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
                        String physType = deviceName + ":" + name;
                        PlatformJNI.JNI_SimSetPhysicsInput(m_type.value, getDeviceID(),
                                                                               physType, getRawValue(value));
                }

                private double getRawValue(HALValue value) {
                  
                    throw new UnsupportedOperationException("Unimplemented method 'getRawValue'");
                }
        }

        private class OnPeriodicCallback implements Runnable {
                @Override
                public void run() {
                        double value = 0;
                        int err = 0;

                        int deviceID = getDeviceID();

                        value = PlatformJNI.JNI_SimGetPhysicsValue(m_type.value, deviceID, "FusedHeading");
                        err = PlatformJNI.JNI_SimGetLastError(m_type.value, deviceID);
                        if (err == 0) {
                                m_simYaw.set(value);
                        }
                        value = PlatformJNI.JNI_SimGetPhysicsValue(m_type.value, deviceID, "HeadingRaw");
                        err = PlatformJNI.JNI_SimGetLastError(m_type.value, deviceID);
                        if (err == 0) {
                                m_simRawYaw.set(value);
                        }
                }
        }


    // ----- WPILib Gyro Interface ----- //
    //WPILib no longer has a Gyro interface, but these methods are standard in FRC.

    /**
     * Resets the Pigeon 2 to a heading of zero.
     * <p>
     * This can be used if there is significant drift in the gyro,
     * and it needs to be recalibrated after it has been running.
     */
    public void reset(){
        setYaw(0);
    }

    /**
     * Returns the heading of the robot in degrees.
     * <p>
     * The angle increases as the Pigeon 2 turns clockwise when looked
     * at from the top. This follows the NED axis convention.
     * <p>
     * The angle is continuous; that is, it will continue from 360 to
     * 361 degrees. This allows for algorithms that wouldn't want to
     * see a discontinuity in the gyro output as it sweeps past from
     * 360 to 0 on the second time around.
     *
     * @return The current heading of the robot in degrees
     */
    public double getAngle(){
        //Negate since getAngle requires cw+ and Pigeon is ccw+
        return -getAngle();
    }

    /**
     * Returns the rate of rotation of the Pigeon 2.
     * <p>
     * The rate is positive as the Pigeon 2 turns clockwise when looked
     * at from the top.
     *
     * @return The current rate in degrees per second
     */
    public double getRate(){
        getRawGyro(_xyz_dps);
        //Expected to return cw+
        return -_xyz_dps[2];
    }

   private void getRawGyro(double[] _xyz_dps2) {
       
        throw new UnsupportedOperationException("Unimplemented method 'getRawGyro'");
    }
/**
     * Returns the heading of the robot as a {@link Rotation2d}.
    * <p>
   * The angle increases as the Pigeon 2 turns counterclockwise when
    * looked at from the top. This follows the NWU axis convention.
     * <p>
     * The angle is continuous; that is, it will continue from 360 to
     * 361 degrees. This allows for algorithms that wouldn't want to
     * see a discontinuity in the gyro output as it sweeps past from
     * 360 to 0 on the second time around.
     *
     * @return The current heading of the robot as a {@link Rotation2d}
     */
    public Rotation2d getRotation2d() {
        //Rotation2d and Pigeon are both ccw+
        return Rotation2d (getYaw());
    }

    private Rotation2d Rotation2d(StatusSignal<Double> yaw) {
   
    throw new UnsupportedOperationException("Unimplemented method 'Rotation2d'");
}
    // ----- Sendable ----- //
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", null, null);
    }
}



