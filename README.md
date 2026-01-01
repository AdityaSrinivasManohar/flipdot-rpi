# Flipdot-rpi
Check out https://www.adityasrinivasmanohar.com/projects/flipdot-display for more details

This project is an attempt to use the [template_repo](https://github.com/AdityaSrinivasManohar/template_repo) to control my flipdot display! Check it out for basic bazel utils and available functions.

To run locally
```
# foxglove bridge and visualizer
bazel run //flipdot/src/foxglove:visualizer
bazel run //flipdot/src/foxglove:foxglove_bridge

# the fsm
bazel run //flipdot/src:flipdot_manager

# simple test scripts
bazel run //flipdot/src:test_publisher
bazel run //flipdot/src:test_subscriber

# To run the controller
bazel run //flipdot/src:controller
```

To deploy to the pi (which is connected to your local network)
```
cd flipdot/scripts

# to deploy packages
./deploy_to_pi.sh --packages

# for setting up systemd services
./deploy_to_pi.sh --systemd

# for cleaning up systemd service
./deploy_to_pi.sh --clean
```

To run all relevant packages on the pi (make sure you are ssh-ed onto the pi)
```
# To run packages
sudo systemctl start flipdot-rpi.target

# To stop packages
sudo systemctl stop flipdot-rpi.target

# To list all dependencies
sudo systemctl stop flipdot-rpi.target
```

Use journalctl on relevant services to view logs
