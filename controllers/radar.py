from controller import Robot, Radar

# Create a Webots robot instance
robot = Robot()

# Initialize the radar
radar = Radar("radar")  # Replace "my_radar" with the actual name of your radar sensor

# Main loop
while robot.step(32) != -1:
    # Access the radar data
    radar_data = radar.getRangeImage()

    # Process the radar data
    for i in range(radar.getHorizontalResolution()):
        angle = radar.getHorizontalAngle(i)
        distance = radar_data[i]
        
        # Print out the detected distances
        print(f"Angle: {angle}, Distance: {distance}")
    
# Cleanup
robot.cleanup()
