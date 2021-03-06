<launch>
  <node pkg="necst" name="antenna_move" type="ROS_antenna_move.py" output="screen" args="Actual_reverse"/>
  <node pkg="necst" name="encoder_status" type="ROS_encoder.py"/>
  <node pkg="necst" name="dome" type="ROS_dome.py"/>
  <node pkg="necst" name="drive" type="ROS_drive.py"/>
  <node pkg="necst" name="limit_check" type="ROS_limit_check.py"/>
  <node pkg="necst" name="antenna_dio" type="ROS_check_antenna_dio.py"/>
  <node pkg="necst" name="dome_dio" type="ROS_check_dome_dio.py"/>
  <node pkg="necst" name="dome_dio" type="ROS_check_encoder_dio.py"/>  
</launch>
