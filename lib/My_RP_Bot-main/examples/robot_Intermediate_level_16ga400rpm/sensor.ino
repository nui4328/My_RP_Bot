int md_sensors(int sensor)
  {
    int sen_md = (sensor_maxs[sensor]+sensor_mins[sensor])/2;
    return sen_md;
  }
