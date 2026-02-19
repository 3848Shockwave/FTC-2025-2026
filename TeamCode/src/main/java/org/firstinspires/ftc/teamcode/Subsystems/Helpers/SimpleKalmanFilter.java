package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

public class SimpleKalmanFilter {
    private double Q; // Process noise covariance (trust in the system model)
    private double R; // Measurement noise covariance (trust in the sensor)
    private double P; // Estimation error covariance
    private double K; // Kalman gain
    private double x; // Value estimate

    public SimpleKalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
        this.P = 1.0;
        this.x = 0.0;
    }

    public double filter(double measurement) {
        P = P + Q;

        K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;

        return x;
    }

    public void setEstimate(double estimate) {
        this.x = estimate;
    }
}
