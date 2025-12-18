사용하려는 world 파일 ( src/turtlebot3_simulations/turtlebot3_gazebo/worlds/empty_world.world 등)
에 아래 항목
<world></world> 안에 추가


<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>
    
    
