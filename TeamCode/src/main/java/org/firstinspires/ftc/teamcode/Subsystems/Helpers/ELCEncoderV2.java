// ========================================
// FILE 1: ELCEncoderV2.java
// This is the encoder class - save this first
// ========================================

package org.firstinspires.ftc.teamcode.Subsystems.Helpers;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.KineticState;

/**
 * ELC Encoder V2 - Supports both Analog and Digital (Quadrature) modes
 */
public class ELCEncoderV2 {

    public enum Mode {
        ANALOG,
        DIGITAL
    }

    private Mode mode;

    // Analog mode variables
    private AnalogInput analogInput;
    private double maxVoltage;
    private double lastDegrees = 0;
    private int rotations = 0;
    private double totalDegrees = 0;

    // Digital mode variables
    private DigitalChannel channelA;
    private DigitalChannel channelB;
    private int position = 0;
    private boolean lastA = false;
    private boolean lastB = false;
    private long lastTimeNs = System.nanoTime();
    private double lastPosition = 0.0;
    /**
     * Constructor for ANALOG mode
     * @param hardwareMap The hardware map from your OpMode
     * @param analogName The name of the analog input in Robot Configuration
     */
    public ELCEncoderV2(HardwareMap hardwareMap, String analogName) {
        this.mode = Mode.ANALOG;
        analogInput = hardwareMap.get(AnalogInput.class, analogName);
        maxVoltage = analogInput.getMaxVoltage();
        lastDegrees = getTotalDegrees();
        totalDegrees = lastDegrees;
    }

    /**
     * Constructor for DIGITAL mode
     * @param hardwareMap The hardware map from your OpMode
     * @param channelAName The name of channel A digital input
     * @param channelBName The name of channel B digital input
     */
    public ELCEncoderV2(HardwareMap hardwareMap, String channelAName, String channelBName) {
        this.mode = Mode.DIGITAL;
        channelA = hardwareMap.get(DigitalChannel.class, channelAName);
        channelB = hardwareMap.get(DigitalChannel.class, channelBName);

        channelA.setMode(DigitalChannel.Mode.INPUT);
        channelB.setMode(DigitalChannel.Mode.INPUT);

        lastA = channelA.getState();
        lastB = channelB.getState();
    }

    /**
     * Get the current mode
     * @return ANALOG or DIGITAL
     */
    public Mode getMode() {
        return mode;
    }

    // ========================================
    // ANALOG MODE METHODS
    // ========================================

    /**
     * Update rotation tracking (ANALOG mode only)
     * Call this repeatedly in your loop to track rotations
     */
    public void updateRotations() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("updateRotations() only works in ANALOG mode");
        }

        double currentDegrees = getDegrees();
        double delta = currentDegrees - lastDegrees;

        // Detect wrap-around
        if (delta > 180) {
            // Wrapped from 360 to 0 (going backwards)
            rotations--;
            delta -= 360;
        } else if (delta < -180) {
            // Wrapped from 0 to 360 (going forwards)
            rotations++;
            delta += 360;
        }

        totalDegrees += delta;
        lastDegrees = currentDegrees;
        lastPosition = currentDegrees;
    }
    public double computeVelocity(double currentPosition) {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9; // seconds
        double vel = 0.0;
        if (dt > 1e-9) {
            vel = (currentPosition - lastPosition) / dt;
        }
        lastPosition = currentPosition;
        lastTimeNs = now;
        return vel;
    }

    /**
     * Reset rotation tracking (ANALOG mode only)
     */
    public void resetRotations() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("resetRotations() only works in ANALOG mode");
        }
        rotations = 0;
        totalDegrees = getDegrees();
        lastDegrees = totalDegrees;
    }

    /**
     * Get number of complete rotations (ANALOG mode only)
     * @return Number of rotations (can be negative)
     */
    public int getRotations() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getRotations() only works in ANALOG mode");
        }
        return rotations;
    }

    /**
     * Get total degrees traveled (ANALOG mode only)
     * @return Total degrees (can exceed 360 or be negative)
     */
    public double getTotalDegrees() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getTotalDegrees() only works in ANALOG mode");
        }
        return totalDegrees;
    }

    /**
     * Get total radians traveled (ANALOG mode only)
     * @return Total radians
     */
    public double getTotalRadians() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getTotalRadians() only works in ANALOG mode");
        }
        return Math.toRadians(totalDegrees);
    }

    /**
     * Get the raw voltage (ANALOG mode only)
     * @return Voltage reading (0 to maxVoltage)
     */
    public double getVoltage() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getVoltage() only works in ANALOG mode");
        }
        return analogInput.getVoltage();
    }

    /**
     * Get the maximum voltage (ANALOG mode only)
     * @return Maximum voltage (typically 3.3V)
     */
    public double getMaxVoltage() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getMaxVoltage() only works in ANALOG mode");
        }
        return maxVoltage;
    }

    /**
     * Get absolute position in degrees (ANALOG mode only)
     * @return Position in degrees (0-360)
     */
    public double getDegrees() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getDegrees() only works in ANALOG mode");
        }
        return (getVoltage() / maxVoltage) * 360.0;
    }

    /**
     * Get absolute position in radians (ANALOG mode only)
     * @return Position in radians (0 to 2π)
     */
    public double getRadians() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getRadians() only works in ANALOG mode");
        }
        return (getVoltage() / maxVoltage) * 2.0 * Math.PI;
    }

    /**
     * Get position as percentage (ANALOG mode only)
     * @return Percentage (0-100)
     */
    public double getPercentage() {
        if (mode != Mode.ANALOG) {
            throw new IllegalStateException("getPercentage() only works in ANALOG mode");
        }
        return (getVoltage() / maxVoltage) * 100.0;
    }

    // ========================================
    // DIGITAL MODE METHODS
    // ========================================

    /**
     * Update the quadrature decoder (DIGITAL mode only)
     * Call this repeatedly in your loop to track position
     */
    public void update() {
        if (mode != Mode.DIGITAL) {
            throw new IllegalStateException("update() only works in DIGITAL mode");
        }

        boolean currentA = channelA.getState();
        boolean currentB = channelB.getState();

        // Quadrature decoding
        if (currentA != lastA) {
            if (currentA == currentB) {
                position++;
            } else {
                position--;
            }
        }

        if (currentB != lastB) {
            if (currentA != currentB) {
                position++;
            } else {
                position--;
            }
        }

        lastA = currentA;
        lastB = currentB;
    }

    /**
     * Get the current position (DIGITAL mode only)
     * @return Position in encoder ticks
     */
    public int getPosition() {
        if (mode != Mode.DIGITAL) {
            throw new IllegalStateException("getPosition() only works in DIGITAL mode");
        }
        return position;
    }
    public KineticState getState(){
        return new KineticState(getTotalDegrees(),computeVelocity(getTotalDegrees()));
    }

    /**
     * Reset the position to zero (DIGITAL mode only)
     */
    public void resetPosition() {
        if (mode != Mode.DIGITAL) {
            throw new IllegalStateException("resetPosition() only works in DIGITAL mode");
        }
        position = 0;
    }

    /**
     * Get channel A state (DIGITAL mode only)
     * @return true if high, false if low
     */
    public boolean getChannelAState() {
        if (mode != Mode.DIGITAL) {
            throw new IllegalStateException("getChannelAState() only works in DIGITAL mode");
        }
        return channelA.getState();
    }

    /**
     * Get channel B state (DIGITAL mode only)
     * @return true if high, false if low
     */
    public boolean getChannelBState() {
        if (mode != Mode.DIGITAL) {
            throw new IllegalStateException("getChannelBState() only works in DIGITAL mode");
        }
        return channelB.getState();
    }
}

