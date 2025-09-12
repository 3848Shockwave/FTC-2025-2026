package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Random;

/**
 * This is a demo OpMode to show how Panels works without requiring a physical robot.
 * It displays a moving dot on the field
 */
@TeleOp(name = "Panels Demo", group = "Demo")
public class PanelsDemo extends OpMode {
    private TelemetryManager telemetryM;
    private FieldManager panelsField;
    
    // For demo animation
    private double x = 0;
    private double y = 0;
    private double angle = 0;
    private Random random = new Random();
    
    // Style for drawing
    private Style robotStyle = new Style("", "#FF5722", 0.0);
    private Style pathStyle = new Style("", "#4CAF50", 0.0);

    @Override
    public void init() {
        // Initialize Panels components
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField = PanelsField.INSTANCE.getField();
        
        // Set up the field with Pedro Pathing offsets
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        
        telemetry.addData("Panels Demo", "Initialized");
        telemetry.addData("Instructions", "Start the OpMode to see Panels in action");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Update telemetry
        telemetryM.debug("Panels Demo");
        telemetryM.debug("Waiting to start...");
        telemetryM.debug("X: " + String.format("%.2f", x));
        telemetryM.debug("Y: " + String.format("%.2f", y));
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        // Reset position when starting
        x = 0;
        y = 0;
        angle = 0;
    }

    @Override
    public void loop() {
        // Animate a moving robot for demo purposes
        angle += 0.1;
        x = 30 * Math.cos(angle);
        y = 30 * Math.sin(angle);
        
        // Add some random telemetry data for demo
        double randomValue = random.nextDouble() * 100;
        
        // Update Panels telemetry
        telemetryM.debug("Panels Demo Running");
        telemetryM.debug("Robot X: " + String.format("%.2f", x));
        telemetryM.debug("Robot Y: " + String.format("%.2f", y));
        telemetryM.debug("Angle: " + String.format("%.2f", Math.toDegrees(angle)));
        telemetryM.debug("Random Value: " + String.format("%.2f", randomValue));
        telemetryM.update(telemetry);
        
        // Draw the robot on the field
        drawRobot(x, y, angle);
        
        // Draw a path trace
        drawPath();
        
        // Send the drawing to Panels
        panelsField.update();
        
        // Also update regular telemetry
        telemetry.addData("Status", "Running Panels Demo");
        telemetry.addData("X", String.format("%.2f", x));
        telemetry.addData("Y", String.format("%.2f", y));
        telemetry.update();
    }
    
    /**
     * Draw a robot representation on the field
     */
    private void drawRobot(double x, double y, double angle) {
        // Draw robot body (circle)
        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(x, y);
        panelsField.circle(9);
        
        // Draw robot heading (line)
        double headX = x + 15 * Math.cos(angle);
        double headY = y + 15 * Math.sin(angle);
        panelsField.moveCursor(x, y);
        panelsField.line(headX, headY);
    }
    
    /**
     * Draw a simple path
     */
    private void drawPath() {
        panelsField.setStyle(pathStyle);
        for (int i = 0; i < 10; i++) {
            double pathX = 20 * Math.cos(i * 0.5);
            double pathY = 20 * Math.sin(i * 0.5);
            if (i == 0) {
                panelsField.moveCursor(pathX, pathY);
            } else {
                panelsField.line(pathX, pathY);
            }
        }
    }
}