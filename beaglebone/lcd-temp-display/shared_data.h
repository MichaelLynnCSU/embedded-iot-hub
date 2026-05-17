#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#define SHM_NAME "/sensor_shm"
#define SEM_NAME "/sensor_sem"

struct HistoryPoint {
    double temp;
    int motion;
    long timestamp;
};

struct Alert {
    long timestamp;
    char message[64];
    int severity;
};

struct RoomStatus {
    int sensor_id;
    char room_name[32];
    char state[16];
    char location[32];
    long timestamp;
};

struct SharedSensorData {
    double current_temp;
    int current_motion;
    long current_timestamp;
    int data_valid;

    double temp_min;
    double temp_max;
    double temp_avg;
    int motion_total;

    struct HistoryPoint history[100];
    int history_count;

    double temp_trend;
    int motion_trend;

    int peak_motion_hour;
    double peak_temp_time;

    int alert_count;
    struct Alert alerts[10];

    int room_count;
    struct RoomStatus rooms[10];

    int total_records;
    long uptime_seconds;
    int disk_usage_percent;

    int last_command;
    int command_result;
    int sequence;
    long response_time_ms;
};

#endif

