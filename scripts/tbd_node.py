#!/usr/bin/env python3

import rclpy
from marmot.tracker import TBDTracker

def main(args=None):
    rclpy.init(args=args)

    # Create & spin tracker object
    tbd_tracker = TBDTracker()
    rclpy.spin(tbd_tracker)

    # Cleanup
    tbd_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()