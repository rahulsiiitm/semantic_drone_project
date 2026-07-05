#!/bin/bash

echo "Spawning a tall red pole directly in front of the drone..."
docker exec -it $(docker ps -q -f ancestor=drone_ros2_env) bash -c 'gz service -s /world/default/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--timeout 1000 \
--req "sdf: \"\
<?xml version=\\\"1.0\\\" ?>\
<sdf version=\\\"1.6\\\">\
  <model name=\\\"test_pole\\\">\
    <pose>5.0 0 1.5 0 0 0</pose>\
    <link name=\\\"link\\\">\
      <collision name=\\\"collision\\\">\
        <geometry>\
          <cylinder><radius>0.2</radius><length>4.0</length></cylinder>\
        </geometry>\
      </collision>\
      <visual name=\\\"visual\\\">\
        <geometry>\
          <cylinder><radius>0.2</radius><length>4.0</length></cylinder>\
        </geometry>\
        <material>\
          <ambient>1 0 0 1</ambient>\
          <diffuse>1 0 0 1</diffuse>\
        </material>\
      </visual>\
    </link>\
  </model>\
</sdf>\""'

echo "Obstacle spawned!"
