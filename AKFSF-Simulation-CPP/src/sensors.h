#ifndef INCLUDE_AKFSFSIM_SENSORS_H
#define INCLUDE_AKFSFSIM_SENSORS_H

#include <random>
#include <vector>

struct GPSMeasurement
{
    double x, y;
};
struct GyroMeasurement
{
    double psi_dot;
};
struct LidarMeasurement
{
    double range, theta;
    int id;
};
struct ImuMeasurement
{
    double ax, ay, psi_dot;
};

class BeaconMap;

class GPSSensor
{
public:
    GPSSensor();
    void reset();
    void setGPSNoiseStd(double std);
    void setGPSErrorProb(double prob);
    void setGPSDeniedZone(double x, double y, double r);
    GPSMeasurement generateGPSMeasurement(double sensor_x, double sensor_y);

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_error_prob;
    double m_gps_denied_x, m_gps_denied_y, m_gps_denied_range;
};

class GyroSensor
{
public:
    GyroSensor();
    void reset();
    void setGyroNoiseStd(double std);
    void setGyroBias(double bias);
    GyroMeasurement generateGyroMeasurement(double sensor_yaw_rate);

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_bias;
};

class ImuSensor
{
public:
    ImuSensor();
    void reset();
    void setAccelNoiseStd(double std);
    void setGyroNoiseStd(double std);
    void setImuGyroBias(double bias);
    void setImuAccelBias(double bias);
    ImuMeasurement generateImuMeasurement(double sensor_accel_x, double sensor_accel_y, double sensor_yaw_rate);

private:
    std::mt19937 m_rand_gen;
    double m_accel_noise_std;
    double m_gyro_noise_std;
    double m_gyro_bias;
    double m_accel_bias;
};

class LidarSensor
{
public:
    LidarSensor();
    void reset();
    void setLidarNoiseStd(double range_std, double theta_std);
    void setLidarMaxRange(double range);
    void setLidarDAEnabled(bool id_enabled);
    std::vector<LidarMeasurement> generateLidarMeasurements(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap &map);

private:
    std::mt19937 m_rand_gen;
    double m_range_noise_std;
    double m_theta_noise_std;
    double m_max_range;
    bool m_id_enabled;
};

#endif // INCLUDE_AKFSFSIM_SENSORS_H