package com.ece475group7.sleepsensor.model

import androidx.annotation.DrawableRes
import androidx.annotation.StringRes

data class SleepSensor (
    @StringRes val sensorNameResourceId: Int,
    @DrawableRes val sensorIconResourceId: Int
)