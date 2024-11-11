package com.ece475group7.sleepsensor.data

import androidx.compose.material.icons.Icons
import com.ece475group7.sleepsensor.R
import com.ece475group7.sleepsensor.model.SleepSensor

class Datasource() {
    fun loadSensors(): List<SleepSensor> {
        return listOf<SleepSensor>(
            SleepSensor(R.string.light_sensor_name, R.drawable.wb_sunny_24px),
            SleepSensor(R.string.accelerometer_sensor_name, R.drawable.timeline_24px),
        )
    }
}
