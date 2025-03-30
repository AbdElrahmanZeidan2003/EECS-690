def avoid_obstacle(self, scan_msg):
    def angle_to_index(angle_rad):
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        return max(0, min(index, len(scan_msg.ranges) - 1))

    # Define the sectors you want to check for escape (back, left, right)
    sectors = {
        'back': [angle_to_index(math.radians(135)), angle_to_index(math.radians(-135))],
        'left': [angle_to_index(math.radians(90))],
        'right': [angle_to_index(math.radians(-90))]
    }

    sector_max = {}
    for name, indices in sectors.items():
        # Get valid readings within each sector
        valid_readings = [(i, scan_msg.ranges[i]) for i in indices
                          if 0.05 < scan_msg.ranges[i] < scan_msg.range_max]
        if valid_readings:
            max_i, max_r = max(valid_readings, key=lambda x: x[1])
            angle = scan_msg.angle_min + max_i * scan_msg.angle_increment
            sector_max[name] = (angle, max_r)

    if sector_max:
        # Find the best sector (i.e., the one with the furthest point)
        best_sector = max(sector_max.items(), key=lambda x: x[1][1])
        best_angle = best_sector[1][0]
        best_distance = best_sector[1][1]

        # Check if there's enough space in the chosen escape direction
        safe_distance_threshold = 0.5  # Define how far in front needs to be clear for safety
        forward_scan_index = angle_to_index(best_angle)  # Index for the forward path
        forward_scan_range = scan_msg.ranges[forward_scan_index]

        if forward_scan_range > safe_distance_threshold:
            # Escape direction is safe, command robot to move
            self.get_logger().info(f'[SAFETY] Escaping toward {best_sector[0]} (angle {best_angle:.2f} rad)')
            twist = Twist()
            twist.linear.x = 0.15  # Forward speed
            twist.angular.z = best_angle  # Rotation angle
            self.cmd_pub.publish(twist)
        else:
            # Escape direction blocked, try another direction
            self.get_logger().warn(f'[SAFETY] Escape blocked! Distance to obstacle: {forward_scan_range:.2f} m')
            # Optionally, try a different direction or rotate in place
            self.publish_safety(True)  # Indicate safety is still engaged
    else:
        self.get_logger().warn('[SAFETY] No valid escape direction found!')
        self.publish_safety(True)  # Indicate safety is still engaged
