def createWorld(useEnclosure=True):
    f = open('simulate.world', 'w+')
    s1 = """<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="simulate-weazelball">
    <physics type="ode">
      <gravity>0 0 -9.80665</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>0</real_time_update_rate>
      <max_contacts>20</max_contacts>
      <simbody>
        <min_step_size>0.0001</min_step_size>
        <accuracy>0.001</accuracy>
        <max_transient_velocity>0.01</max_transient_velocity>
        <contact>
          <stiffness>1e8</stiffness>
          <dissipation>100</dissipation>
          <plastic_coef_restitution>0.5</plastic_coef_restitution>
          <plastic_impact_velocity>0.5</plastic_impact_velocity>
          <static_friction>100</static_friction>
          <dynamic_friction>100</dynamic_friction>
          <viscous_friction>0</viscous_friction>
          <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
          <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
        </contact>
      </simbody>
      <bullet>
        <solver>
          <type>sequential_impulse</type>
          <min_step_size>0.0001</min_step_size>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_surface_layer>0.001</contact_surface_layer>
          <split_impulse>1</split_impulse>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
        </constraints> 
      </bullet>
      <ode>
		<updateRate>-1</updateRate>
        <solver>
          <type>quick</type>
          <min_step_size>0.0001</min_step_size>
          <iters>500</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>1e-8</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <include>
      <uri>model://weazelsun</uri>
    </include>"""

    s2 ="""<include>
      <uri>model://weazelenclosure</uri>
      <pose>0 0 -0.009525 0 0 0</pose>
    </include> 
    """
    s3 ="""<include>
      <uri>model://myGroundPlane</uri>
    </include> 
    <gui>
      <camera name='user_camera'>
<!--        <pose>1.63355 0.021633 0.513402 -3.18134e-20 0.319643 -3.13899</pose> -->
        <pose>0 0 2.0 0 1.57079632 0</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <plugin name='configure' filename='libconfigure.so'> 
    </plugin> 
    <plugin name='StateRecorder' filename='libStateRecorder.so'> 
    </plugin> 
    <plugin name="bumpSensor" filename="libbumpSensor.so"> </plugin>

  </world>
</sdf>"""
    if(useEnclosure):
        f.write(s1+s2+s3)
    else:
        f.write(s1+s3)

if __name__ == "__main__":
    createWorld(False)
