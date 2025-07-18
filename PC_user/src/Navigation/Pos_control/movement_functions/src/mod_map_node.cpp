#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <movement_functions/ModifyMap.h>

// Parseo de string tipo "Z25 0 Z14 45 ..." a pares <zona, ángulo>
std::vector<std::pair<std::string, float>> parseZoneInstructions(const std::string& input) {
    std::istringstream iss(input);
    std::string token;
    std::vector<std::pair<std::string, float>> instructions;

    while (iss >> token) {
        if (token[0] == 'Z') {
            std::string zone = token;
            if (iss >> token) {
                float angle = std::stof(token);
                instructions.emplace_back(zone, angle);
            }
        }
    }
    return instructions;
}

// Callback al recibir el string de zonas y ángulos
void zoneCallback(const std_msgs::String::ConstPtr& msg) {
    std::string instructions_str = msg->data;
    ROS_INFO_STREAM("Recibido: " << instructions_str);

    // 1. Lanzar navegación
    ROS_INFO("Lanzando navigation.launch...");
    system("gnome-terminal -- bash -c 'roslaunch config_files navigation.launch map_name:=logistics-2025.yaml; exec bash'");
    ros::Duration(5.0).sleep();  // tiempo de espera para iniciar

    // 2. Lanzar las TFs de zona
    ROS_INFO("Lanzando TF spawner...");
    system("gnome-terminal -- bash -c 'roslaunch movement_functions logisticsZones_JM.launch; exec bash'");
    ros::Duration(5.0).sleep();

    // 3. Ejecutar el servicio que inicializa reload_amcl_service
    ROS_INFO("Ejecutando reload_amcl_service...");
    system("gnome-terminal -- bash -c 'rosrun movement_functions reload_amcl_service; exec bash'");
    ros::Duration(5.0).sleep();

    // 4. Llamar al servicio /modify_map por cada par ZXX ángulo
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<movement_functions::ModifyMap>("/modify_map");
    movement_functions::ModifyMap srv;

    auto instructions = parseZoneInstructions(instructions_str);

    for (auto& pair : instructions) {
        srv.request.zone_frame = pair.first;
        srv.request.orientation = pair.second;

        ROS_INFO_STREAM("Llamando servicio con zona: " << pair.first << ", ángulo: " << pair.second);
        if (client.call(srv)) {
            ROS_INFO("Modificación aplicada con éxito.");
        } else {
            ROS_WARN("Fallo al llamar al servicio.");
        }
    }

    ros::Duration(2.0).sleep();

    // 5. Matar todos los nodos de navegación
    ROS_INFO("Matando nodos de navegación...");

    std::vector<std::string> nodes_to_kill = {
        "/amcl",
        "/map_server",
        "/prohibition_map_server",
        "/mvn_pln",
        "/map_augmenter",
        "/obs_detector",
        "/simple_move",
        "/path_planner",
        "/rviz"
    };

    for (const auto& node : nodes_to_kill) {
        std::string cmd = "rosnode kill " + node;
        int ret = system(cmd.c_str());
        if (ret != 0) {
            ROS_WARN_STREAM("No se pudo matar: " << node);
        } else {
            ROS_INFO_STREAM("Nodo eliminado: " << node);
        }
    }

    ros::Duration(2.0).sleep();

    // 6. Relanzar navegación con el mapa modificado
    ROS_INFO("Relanzando navegación con mapa modificado...");
    system("gnome-terminal -- bash -c 'roslaunch config_files navigation.launch map_name:=logistics-2025-mod.yaml; exec bash'");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_map_modifier");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zone_modifications", 1, zoneCallback);

    ROS_INFO("Nodo auto_map_modifier iniciado. Esperando datos en /zone_modifications...");
    ros::spin();
    return 0;
}
