package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

/**
 * PanelsSubsystem - Controls the Panels telemetry and field visualization system
 * 
 * This subsystem encapsulates:
 * - Panels telemetry display
 * - Field visualization with robot position and paths
 * - Integration with Pedro Pathing odometry
 */
public class PanelsSubsystem extends SubsystemBase {
    
    // Panels components
    private TelemetryManager telemetryM;
    private FieldManager panelsField;
    
    // Styles for drawing
    private Style robotStyle = new Style("", "#FF5722", 0.0);  // Orange robot
    private Style pathStyle = new Style("", "#4CAF50", 1.0);   // Green path
    private Style targetStyle = new Style("Target", "#2196F3", 2.0); // Blue target
    
    /**
     * Constructor - Initializes Panels components
     */
    public PanelsSubsystem() {
        // Initialize Panels components
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField = PanelsField.INSTANCE.getField();
        
        // Set field offsets for Pedro Pathing
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }
    
    /**
     * Update telemetry with robot information
     * @param robotX X position of robot
     * @param robotY Y position of robot
     * @param heading Robot heading in degrees
     * @param additionalData Any additional data to display
     */
    public void updateTelemetry(double robotX, double robotY, double heading, String... additionalData) {
        telemetryM.debug("Robot X", String.format("%.1f", robotX));
        telemetryM.debug("Robot Y", String.format("%.1f", robotY));
        telemetryM.debug("Heading", String.format("%.1f°", Math.toDegrees(heading)));
        
        // Add any additional data
        for (String data : additionalData) {
            telemetryM.debug(data);
        }
        
        telemetryM.update();
    }
    
    /**
     * Draw the robot on the field
     * @param x X position of robot
     * @param y Y position of robot
     * @param heading Robot heading in radians
     */
    public void drawRobot(double x, double y, double heading) {
        // Draw robot body
        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(x, y);
        panelsField.circle(9);
        
        // Draw heading indicator
        double headX = x + 15 * Math.cos(heading);
        double headY = y + 15 * Math.sin(heading);
        panelsField.moveCursor(x, y);
        panelsField.line(headX, headY);
    }
    
    /**
     * Draw a target on the field
     * @param x X position of target
     * @param y Y position of target
     */
    public void drawTarget(double x, double y) {
        panelsField.setStyle(targetStyle);
        panelsField.moveCursor(x, y);
        panelsField.circle(5);
    }
    
    /**
     * Draw a path between two points
     * @param startX Starting X position
     * @param startY Starting Y position
     * @param endX Ending X position
     * @param endY Ending Y position
     */
    public void drawPath(double startX, double startY, double endX, double endY) {
        panelsField.setStyle(pathStyle);
        panelsField.moveCursor(startX, startY);
        panelsField.line(endX, endY);
    }
    
    /**
     * Send all drawing updates to Panels
     */
    public void updateField() {
        panelsField.update();
    }
    
    /**
     * Get the telemetry manager for direct access if needed
     * @return TelemetryManager instance
     */
    public TelemetryManager getTelemetryManager() {
        return telemetryM;
    }
    
    /**
     * Get the field manager for direct access if needed
     * @return FieldManager instance
     */
    public FieldManager getFieldManager() {
        return panelsField;
    }
}