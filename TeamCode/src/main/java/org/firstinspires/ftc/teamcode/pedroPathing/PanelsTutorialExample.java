package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Panels Tutorial Example - Demonstrates basic Panels functionality
 * This OpMode shows how to use Panels telemetry and field visualization
 */
@TeleOp(name = "Panels Tutorial Example", group = "Pedro Pathing")
public class PanelsTutorialExample extends OpMode {
    private TelemetryManager telemetryM;
    private FieldManager panelsField;
    
    // Styles for drawing
    private Style robotStyle = new Style("", "#FF5722", 0.0);  // Orange robot
    private Style pathStyle = new Style("", "#4CAF50", 1.0);   // Green path
    private Style targetStyle = new Style("Target", "#2196F3", 2.0); // Blue target
    
    // Robot and target positions
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;
    private double targetX = 30;
    private double targetY = 40;
    
    @Override
    public void init() {
        // Initialize Panels components
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField = PanelsField.INSTANCE.getField();
        
        // Set field offsets for Pedro Pathing
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        
        // Display initialization message
        telemetryM.debug("Panels Tutorial Example");
        telemetryM.debug("Use left stick to move robot");
        telemetryM.debug("Use right stick to move target");
        telemetryM.update(telemetry);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Control robot with gamepad
        robotX += gamepad1.left_stick_x * 2;
        robotY += gamepad1.left_stick_y * 2;
        robotHeading += gamepad1.right_stick_x * 0.1;
        
        // Control target with gamepad (right stick)
        targetX += gamepad1.right_stick_x * 2;
        targetY += gamepad1.right_stick_y * 2;
        
        // Calculate distance to target
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);
        
        // Update Panels telemetry
        telemetryM.debug("Robot X", String.format("%.1f", robotX));
        telemetryM.debug("Robot Y", String.format("%.1f", robotY));
        telemetryM.debug("Distance", String.format("%.1f", distance));
        telemetryM.debug("Heading", String.format("%.1f°", Math.toDegrees(robotHeading)));
        telemetryM.update(telemetry);
        
        // Draw path from robot to target
        panelsField.setStyle(pathStyle);
        panelsField.moveCursor(robotX, robotY);
        panelsField.line(targetX, targetY);
        
        // Draw target
        panelsField.setStyle(targetStyle);
        panelsField.moveCursor(targetX, targetY);
        panelsField.circle(5);
        
        // Draw robot
        drawRobot();
        
        // Send update to Panels
        panelsField.update();
        
        // Also update regular telemetry
        telemetry.addData("Robot", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Target", "X: %.1f, Y: %.1f", targetX, targetY);
        telemetry.addData("Distance", "%.1f", distance);
        telemetry.update();
    }
    
    /**
     * Draw the robot on the field with heading indicator
     */
    private void drawRobot() {
        // Draw robot body
        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(robotX, robotY);
        panelsField.circle(9);
        
        // Draw heading indicator
        double headX = robotX + 15 * Math.cos(robotHeading);
        double headY = robotY + 15 * Math.sin(robotHeading);
        panelsField.moveCursor(robotX, robotY);
        panelsField.line(headX, headY);
    }
}