function [] = stop(rc, nhfc, maneuver)
    fprint("Landing")
    maneuver.take_off(0.1, 5) % goto the base height
    rc.stop() % Stop all propellers
    nhfc.stop()
    maneuver.stop()
    %log_stop()
    print("Stop done.")

end