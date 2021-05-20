project(
  'sam',
  'cpp',
  version : '1.0.0',
  default_options : ['cpp_std=c++17']
)

sam_public_headers = include_directories(['src'])

sam_src = [
    'src/main.cpp',
    'src/components/external/myoband/myoLinux/gattclient.cpp',
    'src/components/external/myoband/myoLinux/myoclient.cpp',
    'src/components/external/myoband/myoLinux/serial.cpp',
    'src/components/external/myoband/myoband.cpp',
    'src/components/external/optitrack/optitrack_listener.cpp',
    'src/components/external/ximu/ximu.cpp',
    'src/components/internal/actuators/roboclaw/answer.cpp',
    'src/components/internal/actuators/roboclaw/factory.cpp',
    'src/components/internal/actuators/roboclaw/message.cpp',
    'src/components/internal/actuators/roboclaw/roboclaw.cpp',
    'src/components/internal/actuators/actuator.cpp',
    'src/components/internal/actuators/custom_elbow.cpp',
    'src/components/internal/actuators/osmer_elbow.cpp',
    'src/components/internal/actuators/pronosupination.cpp',
    'src/components/internal/actuators/shoulder_rotator.cpp',
    'src/components/internal/actuators/wrist_flexor.cpp',
    'src/components/internal/actuators/wrist_rotator.cpp',
    'src/components/internal/adc/adafruit_ads1115.cpp',
    'src/components/internal/dac/mcp4728.cpp',
    'src/components/internal/gpio/gpio.cpp',
    'src/components/internal/hand/touch_bionics_hand.cpp',
    'src/control/algo/lawimu.cpp',
    'src/control/algo/lawjacobian.cpp',
    'src/control/algo/lawopti.cpp',
    'src/control/algo/myocontrol.cpp',
    'src/control/compensation_imu.cpp',
    'src/control/compensation_optitrack.cpp',
    'src/control/demo.cpp',
    'src/control/general_formulation.cpp',
    'src/control/matlab_receiver.cpp',
    'src/control/remote_computer_control.cpp',
    'src/control/voluntary_control.cpp',
    'src/sam/sam.cpp',
    'src/sam/samanager.cpp',
    'src/sam/system_monitor.cpp',
    'src/ui/menu/menu_console.cpp',
    'src/ui/menu/menu_frontend.cpp',
    'src/ui/menu/menu_mqtt.cpp',
    'src/ui/sound/buzzer.cpp',
    'src/ui/visual/ledstrip.cpp',
    'src/utils/interfaces/menu_user.cpp',
    'src/utils/interfaces/mqtt_user.cpp',
    'src/utils/log/logger.cpp',
    'src/utils/log/safe_stream.cpp',
    'src/utils/monitoring/abstract_monitor.cpp',
    'src/utils/monitoring/cpu_freq_monitor.cpp',
    'src/utils/monitoring/cpu_load_monitor.cpp',
    'src/utils/monitoring/cpu_temp_monitor.cpp',
    'src/utils/monitoring/vc_based_monitor.cpp',
    'src/utils/named_object.cpp',
    'src/utils/param.cpp',
    'src/utils/serial_port.cpp',
    'src/utils/socket.cpp',
    'src/utils/threaded_loop.cpp',
    'src/utils/watchdog.cpp',
    'src/utils/worker.cpp',
    'src/ux/menu/menu_backend.cpp',
    'src/ux/menu/menu_broker.cpp',
    'src/ux/menu/menu_item.cpp',
    'src/ux/mosquittopp/client.cpp',
    'src/ux/mosquittopp/connect_factory.cpp',
    'src/ux/mosquittopp/connect_helper.cpp',
    'src/ux/mosquittopp/message.cpp',
    'src/ux/mosquittopp/subscription_factory.cpp',
    'src/ux/mosquittopp/subscription.cpp',
]

i2c_dep = declare_dependency(link_args : ['-li2c'])
mosquitto_dep = declare_dependency(link_args : ['-lmosquitto'])
bcm2835_dep = declare_dependency(link_args : ['-lbcm2835'])
vc_dep = declare_dependency(link_args : ['-lvcos', '-lvchiq_arm', '-lvchostif'])
cppfs_dep = declare_dependency(link_args: ['-lstdc++fs'])
thread_dep = dependency('threads')

add_project_arguments('-DMOSQUITTO_SERVER_IP="127.0.0.1"', language : 'cpp')
add_project_arguments('-DMOSQUITTO_SERVER_PORT=1883', language : 'cpp')
add_project_arguments('-DDEFAULT_CPU_CORE=2', language : 'cpp')
add_project_arguments('-DDEFAULT_THREAD_PRIO=20', language : 'cpp')

sam_target = executable('sam', 
    sam_src, 
    include_directories : sam_public_headers, 
    dependencies : [bcm2835_dep, cppfs_dep, i2c_dep, mosquitto_dep, thread_dep, vc_dep],
)
