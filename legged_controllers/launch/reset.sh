rosservice call /gazebo/set_model_state "model_state:
  model_name: 'go1'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.25
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"

rosservice call /gazebo/set_model_configuration '{model_name: "go1", 
  urdf_param_name: "legged_robot_description", 
  joint_names: ["LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"], 
  joint_positions: [-0.5621  1.3056 -2.5357 -0.5621  1.3056 -2.5357  0.5621  1.3056 -2.5357  0.5621  1.3056 -2.5357]}'

