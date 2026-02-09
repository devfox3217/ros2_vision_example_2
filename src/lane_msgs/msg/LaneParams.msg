# HSV Color Thresholds
int32 h_min
int32 h_max
int32 s_min
int32 s_max
int32 v_min
int32 v_max

# Canny Edge Parameters
int32 canny_low
int32 canny_high

# ROI (Region of Interest) Settings (0-100 percentage or pixel values)
# For simplicity, let's assume percentage of height for top cutoff
int32 roi_height_percent

# Perspective Transform offsets (optional)
int32 warp_offset