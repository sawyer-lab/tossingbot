#!/usr/bin/env python3
"""
Landing Detection Demo

Demonstrates contact sensor for detecting object landings.
Shows how to:
- Enable/disable landing detection
- Wait for landing events
- Query landing position and object name
- Use both polling and streaming modes

Requirements:
    - Gazebo simulator with contact sensor configured
    - ZMQ server running in container
    - Object spawned in scene
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from robot_client import ContactSensorClient, GazeboClient
import time


def print_section(title):
    """Print section header."""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def main():
    print_section("Landing Detection Demo")

    print("\n[1] Connecting to sensors...")
    contact = ContactSensorClient(protocol='zmq', host='localhost')
    gazebo = GazeboClient(protocol='zmq', host='localhost')
    print("    Connected")

    print_section("Setup Test Scene")
    
    print("\n   Spawning test object...")
    success = gazebo.spawn('cube', 'test_cube', [0.6, 0.0, 1.5])
    if success:
        print("   Object spawned at height 1.5m")
        print("   It will fall and trigger the landing sensor")
    else:
        print("   Failed to spawn object")
        print("   Demo will test sensor queries only")

    print_section("Contact Sensor Control")
    
    print("\n   Clearing any previous landing data...")
    contact.clear()
    
    print("\n   Starting detection...")
    contact.start_detection()
    print("   Sensor is now active")

    print_section("Wait for Landing (Polling)")
    
    if success:
        print("\n   Waiting for object to land (timeout: 5 seconds)...")
        landing = contact.wait_for_landing(timeout=5.0, poll_rate=50)
        
        if landing:
            pos = landing['position']
            obj = landing['object_name']
            t = landing['timestamp']
            
            print(f"\n   Landing detected:")
            print(f"     Object:   {obj}")
            print(f"     Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"     Time:     {t:.3f}")
        else:
            print("\n   No landing detected (timeout)")
    else:
        print("\n   (Skipped - no object spawned)")

    print_section("Query Last Landing")
    
    print("\n   Querying last landing via get_last_landing()...")
    last = contact.get_last_landing()
    
    if last:
        print(f"   Last landing: {last['object_name']} at {last['position']}")
    else:
        print("   No landing data available")

    print_section("Clear and Stop")
    
    print("\n   Clearing landing data...")
    contact.clear()
    
    verify = contact.get_last_landing()
    print(f"   After clear: {'Data still present!' if verify else 'Cleared successfully'}")
    
    print("\n   Stopping detection...")
    contact.stop_detection()
    print("   Sensor disabled")

    print_section("Cleanup")
    
    if success:
        print("\n   Removing test object...")
        gazebo.despawn('test_cube')
        print("   Cleanup complete")

    print_section("Demo Complete")
    print("\n   Contact sensor functionality demonstrated.")
    print("\n   Key features:")
    print("     - start_detection() / stop_detection()")
    print("     - wait_for_landing() - efficient polling")
    print("     - get_last_landing() - direct query")
    print("     - clear() - reset state")
    print()

    contact.close()
    gazebo.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[Interrupted] Demo stopped")
    except Exception as e:
        print(f"\n\n[ERROR] Demo failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
